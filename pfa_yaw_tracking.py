#!/usr/bin/env python3
"""
PFA Yaw Tracking Task (pymavlink)
====================================
Task: 懸停於高度 1 m，yaw 從初始航向平滑旋轉至目標角度

控制架構：
  Python → SET_POSITION_TARGET_LOCAL_NED (position + yaw = θ(t))
  PX4:  trajectory_setpoint.yaw = θ(t)  (有限值，無 NaN)
        pfa_pos_control: safe_yaw = traj.yaw
          attitude_des = (PFA_DES_ROLL, PFA_DES_PITCH, safe_yaw)
        pfa_att_control → 驅動機體到 yaw = TARGET_YAW_DEG

Yaw 約定 (NED)：
  0° = 朝北 (North)
  +90° = 朝東 (East)
  -90° = 朝西 (West)
  ±180° = 朝南 (South)
  正值 = 順時針旋轉 (viewed from above)

注意：yaw 直接內嵌於位置 setpoint，不需要 PFA_DES_YAW 參數。
      ramp 為連續線性（非離散 PARAM_SET），每 50 ms 更新一次。

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M     = 1.0     # 懸停高度 (m)
TARGET_YAW_DEG   = 90.0    # 目標 yaw (deg, NED: +90=East, 順時針)
TARGET_ROLL_DEG  = 0.0     # roll/pitch 維持 0°
TARGET_PITCH_DEG = 0.0

HOVER_PHASE_DUR  = 8.0     # 水平懸停穩定後等待 (s)
YAW_RAMP_TIME    = 4.0     # yaw ramp 持續時間 (s)
TRACK_PHASE_DUR  = 30.0    # 維持目標 yaw 的追蹤時長 (s)

ALT_TOL          = 0.15
HOVER_STABLE_TIME = 3.0

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting to PX4 (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")
print(f"  Task: alt={TARGET_ALT_M} m, yaw={TARGET_YAW_DEG}° (NED)")


# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────

def param_set(name, value, retries=5):
    name_bytes = name.encode('utf-8')
    for _ in range(retries):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name_bytes, float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if ack and ack.param_id.rstrip('\x00') == name:
            return True
        time.sleep(0.3)
    return False


def set_position_target(x, y, z, yaw_rad=0.0):
    """SET_POSITION_TARGET_LOCAL_NED with explicit finite yaw."""
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000,   # position + yaw (bit10=0), ignore vel/acc/yaw_rate
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
    """Returns (roll, pitch, yaw) in radians (NED convention)."""
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
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0)


def arm(force=False):
    p2 = 21196.0 if force else 0.0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, p2, 0.0, 0.0, 0.0, 0.0, 0.0)


def angle_wrap(a):
    """Wrap angle to [-π, π] (shortest path)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


# ─────────────────────────────────────────────────────────────
# Step 0 – 起飛前確保 roll/pitch 參數歸零
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Configuring PFA attitude target parameters...")
print("  Setting PFA_DES_ROLL  = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED")
print("  Setting PFA_DES_PITCH = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED")

# ─────────────────────────────────────────────────────────────
# Step 1 – Pre-flight check
# ─────────────────────────────────────────────────────────────
print("\n[Step 1] Checking local position estimate...")
x, y, z, _, _, _ = get_local_position()
if x is None:
    print("ERROR: No LOCAL_POSITION_NED.")
    sys.exit(1)
print(f"  OK – NED position: ({x:.2f}, {y:.2f}, {z:.2f})")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
INIT_YAW_RAD = 0.0   # 起飛時送 yaw=0°（朝北），確保有限值

print(f"\n[Step 2] Streaming initial setpoints (5 s, yaw=0°)...")
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=INIT_YAW_RAD)
    time.sleep(0.05)

print("  Requesting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD confirmed.")
else:
    for _ in range(50):
        set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=INIT_YAW_RAD)
        time.sleep(0.05)
    set_mode_offboard()
    print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: not confirmed.")

print("  Arming...")
for _ in range(20):
    set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=INIT_YAW_RAD)
    time.sleep(0.05)
arm()

armed = False
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=INIT_YAW_RAD)
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    arm(force=True)
    for _ in range(100):
        set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=INIT_YAW_RAD)
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
# Step 3 – 位置控制懸停至 1 m (yaw=0°)
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Hover at {TARGET_ALT_M} m  (yaw=0°, PFA_DES_ROLL/PITCH=0°)")
print(f"  Waiting for stable hover (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

phase_start  = time.time()
stable_since = None
last_print   = 0.0

while True:
    set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=INIT_YAW_RAD)
    _, _, pz, _, _, _ = get_local_position()
    if pz is not None:
        alt     = -pz
        alt_err = TARGET_ALT_M - alt
        stable_since = (stable_since or time.time()) if abs(alt_err) < ALT_TOL else None

        now = time.time()
        if now - last_print > 1.0:
            _, _, yaw = get_attitude()
            y_deg = math.degrees(yaw) if yaw is not None else 0.0
            s = now - stable_since if stable_since else 0.0
            print(f"  alt={alt:.2f} m (err={alt_err:+.2f})  yaw={y_deg:.1f}°  "
                  f"stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
            last_print = now

        if stable_since and (time.time() - stable_since) >= HOVER_STABLE_TIME:
            print(f"  Stable hover at {alt:.2f} m.")
            break
        if time.time() - phase_start > 25.0:
            print(f"  Timeout – alt={alt:.2f} m, continuing.")
            break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 4 – 水平懸停保持 HOVER_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
end_t      = time.time() + HOVER_PHASE_DUR
last_print = 0.0
while time.time() < end_t:
    set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=INIT_YAW_RAD)
    now = time.time()
    if now - last_print > 2.0:
        _, _, pz, _, _, _ = get_local_position()
        _, _, yaw = get_attitude()
        alt   = -pz if pz is not None else 0.0
        y_deg = math.degrees(yaw) if yaw is not None else 0.0
        print(f"  alt={alt:.2f} m  yaw={y_deg:.1f}°  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 5 – Yaw ramp：連續線性旋轉至 TARGET_YAW_DEG
#
#  yaw 直接內嵌於 SET_POSITION_TARGET_LOCAL_NED.yaw，
#  pfa_pos_control 讀取 trajectory_setpoint.yaw → safe_yaw → attitude_des.z
#  無需 PARAM_SET，每 50 ms 更新一次。
# ─────────────────────────────────────────────────────────────
_, _, yaw_now = get_attitude()
initial_yaw_rad = yaw_now if yaw_now is not None else 0.0
target_yaw_rad  = math.radians(TARGET_YAW_DEG)
delta_yaw_rad   = angle_wrap(target_yaw_rad - initial_yaw_rad)

print(f"\n[Step 5] Yaw ramp: {math.degrees(initial_yaw_rad):.1f}° → {TARGET_YAW_DEG:.1f}°  "
      f"(delta={math.degrees(delta_yaw_rad):+.1f}°, duration={YAW_RAMP_TIME:.1f} s)")
print("  → yaw 內嵌於 SET_POSITION_TARGET_LOCAL_NED（無 PARAM_SET）")

ramp_start = time.time()
last_print = 0.0

while True:
    elapsed  = time.time() - ramp_start
    frac     = min(elapsed / YAW_RAMP_TIME, 1.0)
    cmd_yaw  = initial_yaw_rad + frac * delta_yaw_rad

    set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=cmd_yaw)

    now = time.time()
    if now - last_print > 0.5:
        _, _, pz, _, _, _ = get_local_position()
        _, _, yaw_fb = get_attitude()
        alt   = -pz if pz is not None else 0.0
        y_deg = math.degrees(yaw_fb) if yaw_fb is not None else 0.0
        print(f"  alt={alt:.2f} m  yaw_cmd={math.degrees(cmd_yaw):.1f}°  yaw_now={y_deg:.1f}°")
        last_print = now

    if frac >= 1.0:
        break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 6 – 維持 yaw=TARGET_YAW_DEG, alt=1 m，追蹤 TRACK_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 6] Holding yaw={TARGET_YAW_DEG:.1f}°, "
      f"alt={TARGET_ALT_M} m for {TRACK_PHASE_DUR:.0f} s...")
print(f"  {'Time':>6}  {'Alt':>6}  {'AltErr':>7}  {'YawCmd':>8}  {'YawNow':>8}")
print("  " + "─" * 44)

track_start = time.time()
last_print  = 0.0

while time.time() - track_start < TRACK_PHASE_DUR:
    set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=target_yaw_rad)
    _, _, pz, _, _, _ = get_local_position()
    _, _, yaw_fb = get_attitude()
    alt     = -pz if pz is not None else TARGET_ALT_M
    y_deg   = math.degrees(yaw_fb) if yaw_fb is not None else 0.0
    elapsed = time.time() - track_start
    now     = time.time()
    if now - last_print >= 0.5:
        print(f"  {elapsed:6.1f}s  {alt:6.2f}m  {TARGET_ALT_M-alt:+7.2f}m  "
              f"{TARGET_YAW_DEG:7.1f}°  {y_deg:7.1f}°")
        last_print = now
    time.sleep(0.05)

print("\n  Yaw tracking complete.")

# ─────────────────────────────────────────────────────────────
# Step 7 – 降落
# ─────────────────────────────────────────────────────────────
print("\n[Step 7] Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
