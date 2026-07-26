#!/usr/bin/env python3
"""
PFA Figure-8 Path Tracking (pymavlink)
=======================================
Task: 繞 8 字軌跡飛行，roll=pitch=0°，yaw 跟隨前進切線方向

軌跡 (NED，以懸停位置為 8 字中心)：
  Lissajous 1:2 參數曲線：
    x(t) = cx + A × sin(ω t)           A = FIG8_LONG_RADIUS  (南北長軸)
    y(t) = cy + B × sin(2ω t)          B = FIG8_LONG_RADIUS / 2 (東西短軸)
    z    = -TARGET_ALT_M               (定高)
    ω    = 2π / PERIOD_S

  t=0        → (cx, cy)      前進方向：NE（往北半圓出發）
  t=T/4      → (cx+A, cy)   最北點，前進方向：E → S
  t=T/2      → (cx, cy)     中心交叉，前進方向：SW（進入南半圓）
  t=3T/4     → (cx-A, cy)   最南點，前進方向：W → N
  t=T        → (cx, cy)     中心，完成一圈

最大速度（中心交叉時）：
    |v_max| = ω × sqrt(A² + 4B²)  （當 B=A/2 → A·ω·√2）

控制架構：
  Python → SET_POSITION_TARGET_LOCAL_NED (x=x(t), y=y(t), z=-ALT, yaw)
  PX4:  pfa_pos_control → thrust（追蹤 8 字位置）
        pfa_att_control → roll=0°, pitch=0°, yaw 跟切線

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 5.0    # 飛行高度 (m)
FIG8_LONG_RADIUS  = 10.0   # 8字南北長軸半徑 A (m); 短軸 B = A/2
PERIOD_S          = 30.0   # 一個完整 8 字的時間 (s)
NUM_LAPS          = 3      # 飛行完整 8 字圈數
YAW_TRACK_PATH    = True   # True: yaw 跟切線方向; False: 固定 yaw=0° (NED North)

HOVER_PHASE_DUR   = 8.0
ALT_TOL           = 0.15
HOVER_STABLE_TIME = 3.0

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
FIG8_SHORT_RADIUS = FIG8_LONG_RADIUS / 2.0
OMEGA             = 2.0 * math.pi / PERIOD_S
TOTAL_DUR_S       = NUM_LAPS * PERIOD_S

V_CENTER_MPS = OMEGA * math.sqrt(FIG8_LONG_RADIUS**2 + (2*FIG8_SHORT_RADIUS)**2)
V_LOBE_MPS   = FIG8_SHORT_RADIUS * 2.0 * OMEGA

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}")
print(f"  Figure-8: A={FIG8_LONG_RADIUS:.1f} m (N/S), B={FIG8_SHORT_RADIUS:.1f} m (E/W), "
      f"T={PERIOD_S:.0f} s/lap × {NUM_LAPS} = {TOTAL_DUR_S:.0f} s")
print(f"  Speed: center={V_CENTER_MPS:.2f} m/s, lobe tips={V_LOBE_MPS:.2f} m/s")
print(f"  Yaw: {'切線方向 (atan2)' if YAW_TRACK_PATH else '固定 0° (NED North)'}")


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
    """NED position + yaw setpoint (ignore vel/acc/yaw_rate)."""
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000,
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


def fig8_yaw(elapsed):
    """NED tangent yaw for Lissajous 1:2. Continuous at center crossings.
    vx_ned = A*ω*cos(ωt),  vy_ned = 2B*ω*cos(2ωt)
    yaw_ned = atan2(vy, vx)  (0=North, +π/2=East)
    """
    vx = FIG8_LONG_RADIUS  * OMEGA       * math.cos(OMEGA * elapsed)
    vy = FIG8_SHORT_RADIUS * 2.0 * OMEGA * math.cos(2.0 * OMEGA * elapsed)
    if abs(vx) < 1e-9 and abs(vy) < 1e-9:
        return 0.0
    return math.atan2(vy, vx)


# ─────────────────────────────────────────────────────────────
# Step 0 – PFA attitude parameters
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0°...")
print("  PFA_DES_ROLL  = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED")
print("  PFA_DES_PITCH = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED")

# ─────────────────────────────────────────────────────────────
# Step 1 – Pre-flight → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
print("\n[Step 1] Pre-flight → OFFBOARD → Arm...")
x0, y0, z0, _, _, _ = get_local_position()
if x0 is None:
    print("  ERROR: No LOCAL_POSITION_NED.")
    sys.exit(1)
print(f"  NED position: ({x0:.2f}, {y0:.2f}, {z0:.2f})")

print(f"  Streaming setpoints (5 s, z={-TARGET_ALT_M:.1f})...")
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)

set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD confirmed.")
else:
    for _ in range(50):
        set_position_target(0.0, 0.0, -TARGET_ALT_M)
        time.sleep(0.05)
    set_mode_offboard()
    print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: not confirmed.")

for _ in range(20):
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)
arm()

armed = False
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    arm(force=True)
    for _ in range(100):
        set_position_target(0.0, 0.0, -TARGET_ALT_M)
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.05)

if not armed:
    print("  ERROR: Failed to arm.")
    sys.exit(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stable hover at TARGET_ALT_M
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Stable hover at {TARGET_ALT_M} m...")
phase_start  = time.time()
stable_since = None
last_print   = 0.0

while True:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    _, _, pz, _, _, _ = get_local_position()
    if pz is not None:
        alt     = -pz
        alt_err = TARGET_ALT_M - alt
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
# Step 3 – Hover hold; record figure-8 center
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Hover hold for {HOVER_PHASE_DUR:.0f} s (recording 8-center)...")
end_t      = time.time() + HOVER_PHASE_DUR
last_print = 0.0
while time.time() < end_t:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        _, _, pz, _, _, _ = get_local_position()
        print(f"  alt={-pz:.2f} m  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

px0, py0, _, _, _, _ = get_local_position()
cx = px0 if px0 is not None else 0.0
cy = py0 if py0 is not None else 0.0
print(f"  Figure-8 center: NED ({cx:.2f}, {cy:.2f}) m")
print(f"  North lobe: x ≈ [{cx:.1f}, {cx+FIG8_LONG_RADIUS:.1f}] m  "
      f"South lobe: x ≈ [{cx-FIG8_LONG_RADIUS:.1f}, {cx:.1f}] m")

# ─────────────────────────────────────────────────────────────
# Step 4 – Figure-8 trajectory
# ─────────────────────────────────────────────────────────────
initial_yaw = fig8_yaw(0.0)

print(f"\n[Step 4] Figure-8: {NUM_LAPS} laps × {PERIOD_S:.0f} s = {TOTAL_DUR_S:.0f} s")
print(f"  Initial yaw command: {math.degrees(initial_yaw):.1f}°")
print(f"  {'Time':>6}  {'Lap':>4}  {'Xcmd':>7}  {'Ycmd':>7}  "
      f"{'Xnow':>7}  {'Ynow':>7}  {'Alt':>5}  {'YawCmd':>8}")
print("  " + "─" * 68)

t_track    = time.time()
last_print = 0.0
yaw_cmd    = initial_yaw

while True:
    now     = time.time()
    elapsed = now - t_track
    if elapsed >= TOTAL_DUR_S:
        elapsed = TOTAL_DUR_S

    x_cmd = cx + FIG8_LONG_RADIUS  * math.sin(OMEGA * elapsed)
    y_cmd = cy + FIG8_SHORT_RADIUS * math.sin(2.0 * OMEGA * elapsed)

    if YAW_TRACK_PATH:
        yaw_cmd = fig8_yaw(elapsed)

    set_position_target(x_cmd, y_cmd, -TARGET_ALT_M, yaw_rad=yaw_cmd)

    if now - last_print >= 0.5:
        px, py, pz, _, _, _ = get_local_position()
        xn  = px if px is not None else x_cmd
        yn  = py if py is not None else y_cmd
        alt = -pz if pz is not None else TARGET_ALT_M
        lap_num = elapsed / PERIOD_S
        print(f"  {elapsed:6.1f}s  {lap_num:4.2f}  {x_cmd:7.2f}m  {y_cmd:7.2f}m  "
              f"{xn:7.2f}m  {yn:7.2f}m  {alt:5.2f}m  "
              f"{math.degrees(yaw_cmd):7.1f}°")
        last_print = now

    if elapsed >= TOTAL_DUR_S:
        break
    time.sleep(0.05)

print("\n  Figure-8 complete.")

# ─────────────────────────────────────────────────────────────
# Step 5 – Return to center and hold 5 s
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 5] Returning to center ({cx:.1f}, {cy:.1f}) m, hold 5 s...")
end_t      = time.time() + 5.0
last_print = 0.0
while time.time() < end_t:
    set_position_target(cx, cy, -TARGET_ALT_M, yaw_rad=0.0)
    now = time.time()
    if now - last_print > 1.0:
        px, py, pz, _, _, _ = get_local_position()
        roll, pitch, _ = get_attitude()
        alt   = -pz         if pz    is not None else TARGET_ALT_M
        r_deg = math.degrees(roll)  if roll  is not None else 0.0
        p_deg = math.degrees(pitch) if pitch is not None else 0.0
        print(f"  NED=({px:.2f}, {py:.2f})  alt={alt:.2f} m  "
              f"roll={r_deg:.1f}°  pitch={p_deg:.1f}°")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 6 – Land
# ─────────────────────────────────────────────────────────────
print("\n[Step 6] Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
