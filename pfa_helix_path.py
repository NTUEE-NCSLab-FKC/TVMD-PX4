#!/usr/bin/env python3
"""
PFA Helix Path Tracking (pymavlink)
=====================================
Task: 追蹤螺旋上升軌跡，roll=pitch=0°，yaw 跟隨切線方向

軌跡 (NED，俯視為逆時針)：
  x(t) = x_c + R × cos(2π t / T_rev)    (NED North 分量)
  y(t) = y_c + R × sin(2π t / T_rev)    (NED East  分量)
  z(t) = -(h₀ + t × CLIMB_RATE_MPS)     (z 遞減 = 高度遞增)
  yaw(t) = atan2(cos(θ), -sin(θ))        (切線方向，始終朝向前進方向)

θ(t) = 2π t / T_rev (螺旋相位角)

物理量：
  切線速度  v_tan = 2π R / T_rev
  向心加速度 a_c  = (2π R / T_rev)² / R = 4π² R / T_rev²

控制架構：
  Python → SET_POSITION_TARGET_LOCAL_NED (x, y, z, yaw = 螺旋公式)
  PX4:  pfa_pos_control → thrust (追蹤 3D 螺旋位置)
                        → attitude_des = (0, 0, safe_yaw)   ← PFA_DES_ROLL/PITCH=0
        pfa_att_control → roll=0°, pitch=0°, yaw 跟隨切線

步驟：
  0. 參數歸零  1. OFFBOARD+Arm  2. 懸停  3. 平飛保持
  4. 接近螺旋起點 (x_c+R, y_c)
  5. 螺旋上升  6. 保持終點  7. 降落

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_START_ALT_M  = 1.0    # 螺旋起始高度 (m)
HELIX_RADIUS_M      = 1.5    # 螺旋半徑 (m)
CLIMB_RATE_MPS      = 0.3    # 垂直爬升速率 (m/s)
REV_PERIOD_S        = 8.0    # 每圈時間 (s/rev)
NUM_REVOLUTIONS     = 3      # 總圈數
YAW_TRACK_PATH      = True   # True: yaw 跟切線; False: yaw=0° 固定

HOVER_PHASE_DUR     = 8.0
APPROACH_DUR_S      = 4.0    # 從懸停中心移動到螺旋起點的時間 (s)
ALT_TOL             = 0.15
HOVER_STABLE_TIME   = 3.0

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
TOTAL_DUR_S     = NUM_REVOLUTIONS * REV_PERIOD_S
TOTAL_CLIMB_M   = CLIMB_RATE_MPS * TOTAL_DUR_S
FINAL_ALT_M     = TARGET_START_ALT_M + TOTAL_CLIMB_M
V_TAN_MPS       = 2.0 * math.pi * HELIX_RADIUS_M / REV_PERIOD_S
A_CENTRIPETAL   = V_TAN_MPS ** 2 / HELIX_RADIUS_M
OMEGA           = 2.0 * math.pi / REV_PERIOD_S

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}")
print(f"  Helix: R={HELIX_RADIUS_M} m, T={REV_PERIOD_S} s/rev, "
      f"{NUM_REVOLUTIONS} revs, climb={CLIMB_RATE_MPS} m/s")
print(f"  v_tan={V_TAN_MPS:.2f} m/s, a_centripetal={A_CENTRIPETAL:.3f} m/s²")
print(f"  Alt: {TARGET_START_ALT_M:.1f} m → {FINAL_ALT_M:.1f} m "
      f"(+{TOTAL_CLIMB_M:.1f} m in {TOTAL_DUR_S:.0f} s)")
print(f"  Attitude: roll=0°, pitch=0°, "
      f"{'yaw=切線方向' if YAW_TRACK_PATH else 'yaw=0° 固定'}")


# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────

def param_set(name, value, retries=5):
    nb = name.encode('utf-8')
    for _ in range(retries):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            nb, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if ack and ack.param_id.rstrip('\x00') == name:
            return True
        time.sleep(0.3)
    return False


def set_position_target(x, y, z, yaw_rad=0.0):
    """NED position + yaw setpoint."""
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000,   # position + yaw; ignore vel/acc/yaw_rate
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        float(yaw_rad), 0)


def get_local_position():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg:
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
    return None, None, None, None, None, None


def get_attitude():
    """Returns (roll, pitch, yaw) in radians."""
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        return msg.roll, msg.pitch, msg.yaw
    return None, None, None


def wait_for_mode(target_main_mode, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and ((msg.custom_mode >> 16) & 0xFF) == target_main_mode:
            return True
    return False


def set_mode_offboard():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        6.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def set_mode_land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def arm(force=False):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0 if force else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def helix_yaw(theta):
    """Yaw aligned to helix tangent (NED).
    d/dt[R*cos(θ), R*sin(θ)] = [-R*ω*sin(θ), R*ω*cos(θ)]
    yaw = atan2(vy_NED, vx_NED) = atan2(cos θ, -sin θ)
    """
    return math.atan2(math.cos(theta), -math.sin(theta))


# ─────────────────────────────────────────────────────────────
# Step 0 – PFA_DES_ROLL/PITCH = 0°
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0° (level flight)...")
print("  PFA_DES_ROLL  = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED")
print("  PFA_DES_PITCH = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED")

# ─────────────────────────────────────────────────────────────
# Step 1 – Pre-flight → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
print("\n[Step 1] Pre-flight check...")
x, y, z, _, _, _ = get_local_position()
if x is None:
    print("ERROR: No LOCAL_POSITION_NED.")
    sys.exit(1)
print(f"  NED position: ({x:.2f}, {y:.2f}, {z:.2f})")

print(f"\n  Streaming setpoints (5 s)...")
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_START_ALT_M)
    time.sleep(0.05)

print("  Requesting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD confirmed.")
else:
    for _ in range(50):
        set_position_target(0.0, 0.0, -TARGET_START_ALT_M)
        time.sleep(0.05)
    set_mode_offboard()
    print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: not confirmed.")

print("  Arming...")
for _ in range(20):
    set_position_target(0.0, 0.0, -TARGET_START_ALT_M)
    time.sleep(0.05)
arm()

armed = False
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_START_ALT_M)
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    arm(force=True)
    for _ in range(100):
        set_position_target(0.0, 0.0, -TARGET_START_ALT_M)
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.05)

if not armed:
    print("ERROR: Failed to arm.")
    sys.exit(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 2 – 穩定懸停至 TARGET_START_ALT_M
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Stable hover at {TARGET_START_ALT_M} m...")
phase_start  = time.time()
stable_since = None
last_print   = 0.0

while True:
    set_position_target(0.0, 0.0, -TARGET_START_ALT_M)
    _, _, pz, _, _, _ = get_local_position()
    if pz is not None:
        alt     = -pz
        alt_err = TARGET_START_ALT_M - alt
        stable_since = (stable_since or time.time()) if abs(alt_err) < ALT_TOL else None
        now = time.time()
        if now - last_print > 1.0:
            s = now - stable_since if stable_since else 0.0
            print(f"  alt={alt:.2f} m (err={alt_err:+.2f})  "
                  f"stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
            last_print = now
        if stable_since and (time.time() - stable_since) >= HOVER_STABLE_TIME:
            print(f"  Stable at {alt:.2f} m.")
            break
        if time.time() - phase_start > 25.0:
            print(f"  Timeout – alt={alt:.2f} m, continuing.")
            break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 3 – Level hover hold
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
end_t      = time.time() + HOVER_PHASE_DUR
last_print = 0.0
while time.time() < end_t:
    set_position_target(0.0, 0.0, -TARGET_START_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        _, _, pz, _, _, _ = get_local_position()
        print(f"  alt={-pz:.2f} m  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# Record helix center = current position
px0, py0, _, _, _, _ = get_local_position()
hx = px0 if px0 is not None else 0.0   # helix center x (NED)
hy = py0 if py0 is not None else 0.0   # helix center y (NED)
print(f"  Helix center: NED ({hx:.2f}, {hy:.2f}) m")

# ─────────────────────────────────────────────────────────────
# Step 4 – 接近螺旋起點 (hx+R, hy)
# ─────────────────────────────────────────────────────────────
helix_start_yaw = helix_yaw(0.0)   # yaw at theta=0
print(f"\n[Step 4] Approaching helix start ({hx+HELIX_RADIUS_M:.2f}, {hy:.2f}) m "
      f"in {APPROACH_DUR_S:.0f} s...")

approach_start = time.time()
while True:
    now     = time.time()
    elapsed = now - approach_start
    frac    = min(elapsed / APPROACH_DUR_S, 1.0)

    # Smooth cubic ease-in-out to helix start
    frac_smooth = frac * frac * (3.0 - 2.0 * frac)
    x_cmd = hx + frac_smooth * HELIX_RADIUS_M
    y_cmd = hy
    yaw_cmd = helix_start_yaw if YAW_TRACK_PATH else 0.0

    set_position_target(x_cmd, y_cmd, -TARGET_START_ALT_M, yaw_rad=yaw_cmd)

    now = time.time()
    if now - last_print > 1.0:
        px, _, pz, _, _, _ = get_local_position()
        print(f"  x={px:.2f} m (cmd={x_cmd:.2f})  alt={-pz:.2f} m  "
              f"frac={frac*100:.0f}%")
        last_print = now

    if frac >= 1.0:
        break
    time.sleep(0.05)

print("  At helix start.")

# ─────────────────────────────────────────────────────────────
# Step 5 – 螺旋上升
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 5] Helix ascent: {NUM_REVOLUTIONS} revs × {REV_PERIOD_S:.0f} s = "
      f"{TOTAL_DUR_S:.0f} s, {TARGET_START_ALT_M:.1f}→{FINAL_ALT_M:.1f} m")
print(f"  {'Time':>6}  {'θ(°)':>6}  {'Xcmd':>6}  {'Ycmd':>6}  "
      f"{'AltCmd':>7}  {'AltNow':>7}  {'YawCmd':>8}  {'Roll':>6}  {'Pitch':>6}")
print("  " + "─" * 76)

track_start = time.time()
last_print  = 0.0

while True:
    now     = time.time()
    elapsed = now - track_start
    if elapsed >= TOTAL_DUR_S:
        elapsed = TOTAL_DUR_S

    theta   = OMEGA * elapsed                          # radians
    x_cmd   = hx + HELIX_RADIUS_M * math.cos(theta)
    y_cmd   = hy + HELIX_RADIUS_M * math.sin(theta)
    z_cmd   = -(TARGET_START_ALT_M + elapsed * CLIMB_RATE_MPS)
    yaw_cmd = helix_yaw(theta) if YAW_TRACK_PATH else 0.0

    set_position_target(x_cmd, y_cmd, z_cmd, yaw_rad=yaw_cmd)

    if now - last_print >= 0.5:
        px, _, pz, _, _, _ = get_local_position()
        roll, pitch, yaw_fb = get_attitude()
        alt_now = -pz if pz is not None else -z_cmd
        r_deg   = math.degrees(roll)    if roll   is not None else 0.0
        p_deg   = math.degrees(pitch)   if pitch  is not None else 0.0
        print(f"  {elapsed:6.1f}s  {math.degrees(theta)%360:6.1f}°  "
              f"{x_cmd:6.2f}m  {y_cmd:6.2f}m  "
              f"{-z_cmd:7.2f}m  {alt_now:7.2f}m  "
              f"{math.degrees(yaw_cmd):7.1f}°  {r_deg:5.1f}°  {p_deg:5.1f}°")
        last_print = now

    if elapsed >= TOTAL_DUR_S:
        break
    time.sleep(0.05)

print("\n  Helix ascent complete.")

# ─────────────────────────────────────────────────────────────
# Step 6 – Hold end position for 10 s
# ─────────────────────────────────────────────────────────────
px_end, py_end, pz_end, _, _, _ = get_local_position()
x_hold = px_end if px_end is not None else hx + HELIX_RADIUS_M
y_hold = py_end if py_end is not None else hy
z_hold = pz_end if pz_end is not None else -(TARGET_START_ALT_M + TOTAL_CLIMB_M)
final_yaw = helix_yaw(OMEGA * TOTAL_DUR_S) if YAW_TRACK_PATH else 0.0

print(f"\n[Step 6] Holding end position ({x_hold:.2f}, {y_hold:.2f}, "
      f"alt={-z_hold:.2f} m) for 10 s...")
end_t      = time.time() + 10.0
last_print = 0.0
while time.time() < end_t:
    set_position_target(x_hold, y_hold, z_hold, yaw_rad=final_yaw)
    now = time.time()
    if now - last_print > 2.0:
        px, _, pz, _, _, _ = get_local_position()
        roll, pitch, _ = get_attitude()
        alt   = -pz if pz is not None else -z_hold
        r_deg = math.degrees(roll)  if roll  is not None else 0.0
        p_deg = math.degrees(pitch) if pitch is not None else 0.0
        print(f"  alt={alt:.2f} m  roll={r_deg:.1f}°  pitch={p_deg:.1f}°  "
              f"remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 7 – Land
# ─────────────────────────────────────────────────────────────
print("\n[Step 7] Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(20)
print("Done.")
