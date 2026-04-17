#!/usr/bin/env python3
"""
PFA Roll Tracking Task (pymavlink)
====================================
Based on pfa_attitude_tracking.py — roll 版本

Task: 懸停於高度 1 m，roll 維持 45 度

控制架構（與 pitch 版完全對稱）：
  Python → PARAM_SET PFA_DES_ROLL = φ(t)
  PX4:  attitude_des = (PFA_DES_ROLL, PFA_DES_PITCH, safe_yaw)
        thrust_body  = rotateVectorInverse(PD_force_NED, q_current)
        → pfa_att_control 驅動機體到 roll=45°

Roll 物理分析（對應 pitch 版的 body-X → body-Y）：
  F_body_y = hover_thrust × sin(roll) = 0.572 × sin(roll)
  _thrust_xy_max = 0.3
  飽和角 = arcsin(0.3/0.572) ≈ ±31.6°

  roll=30° → F_body_y=0.286 < 0.3  ✓
  roll=45° → F_body_y=0.405 > 0.3  ✗ (XY 板手飽和，位置漂移)

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M     = 1.0        # 懸停高度 (m)
TARGET_ROLL_DEG  = 45.0       # 期望 roll (deg，正值 = 右翼下)
TARGET_PITCH_DEG = 0.0        # pitch 維持水平

HOVER_PHASE_DUR  = 8.0
PITCH_RAMP_TIME  = 4.0        # roll ramp 時間 (s)
TRACK_PHASE_DUR  = 30.0

PARAM_STEP_DEG   = 5.0        # 每步進量 (deg，恆正)

ALT_TOL          = 0.15
HOVER_STABLE_TIME = 3.0

# XY 飽和限制提示
_HOVER_THR = (1.4 * 9.81) / 24.0
_SAT_LIMIT = math.degrees(math.asin(0.3 / _HOVER_THR))   # ≈ 31.6°

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting to PX4 (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")
print(f"  Task: alt={TARGET_ALT_M} m, roll={TARGET_ROLL_DEG}°")


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


def param_get(name):
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), -1)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg and msg.param_id.rstrip('\x00') == name:
        return msg.param_value
    return None


def set_position_target(x, y, z):
    """SET_POSITION_TARGET_LOCAL_NED, position + yaw=0 (有限值，防 NaN)."""
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000,   # position + yaw (bit10=0)，ignore vel/acc/yaw_rate
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0.0, 0)               # yaw=0.0 (有限值)


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
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0)


def arm(force=False):
    p2 = 21196.0 if force else 0.0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, p2, 0.0, 0.0, 0.0, 0.0, 0.0)


# ─────────────────────────────────────────────────────────────
# Step 0 – 參數初始化
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Configuring PFA attitude target parameters...")
val = param_get('PFA_DES_ROLL')
print(f"  PFA_DES_ROLL  current = {val:.1f}°" if val is not None
      else "  PFA_DES_ROLL: read failed")

print("  Setting PFA_DES_PITCH = 0°  ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED")
print("  Setting PFA_DES_ROLL  = 0°  ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED")

# ─────────────────────────────────────────────────────────────
# Step 1 – Pre-flight
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
print(f"\n[Step 2] Streaming initial setpoints (5 s)...")
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)

print("  Requesting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD confirmed.")
else:
    for _ in range(50):
        set_position_target(0.0, 0.0, -TARGET_ALT_M)
        time.sleep(0.05)
    set_mode_offboard()
    print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: not confirmed.")

print("  Arming...")
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
    print("ERROR: Failed to arm.")
    sys.exit(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – 位置控制懸停至 1 m
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Hover at {TARGET_ALT_M} m  (PFA_DES_ROLL=0°, pfa_pos_control)")
print(f"  Waiting for stable hover (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

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
            roll, _, _ = get_attitude()
            r_deg = math.degrees(roll) if roll is not None else 0.0
            s = now - stable_since if stable_since else 0.0
            print(f"  alt={alt:.2f} m (err={alt_err:+.2f})  roll={r_deg:.1f}°  "
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
# Step 4 – 水平懸停保持
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
end_t    = time.time() + HOVER_PHASE_DUR
last_print = 0.0
while time.time() < end_t:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        _, _, pz, _, _, _ = get_local_position()
        roll, _, _ = get_attitude()
        alt   = -pz if pz is not None else 0.0
        r_deg = math.degrees(roll) if roll is not None else 0.0
        print(f"  alt={alt:.2f} m  roll={r_deg:.1f}°  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 5 – Roll ramp：透過 PARAM_SET PFA_DES_ROLL 漸進傾斜
# ─────────────────────────────────────────────────────────────
PARAM_STEP_WAIT = PITCH_RAMP_TIME / (abs(TARGET_ROLL_DEG) / PARAM_STEP_DEG)

print(f"\n[Step 5] Roll ramp: 0° → {TARGET_ROLL_DEG:.0f}° "
      f"(step={PARAM_STEP_DEG:.0f}°, interval={PARAM_STEP_WAIT:.1f} s/step)")
print(f"  XY thrust saturation limit: ±{_SAT_LIMIT:.1f}°  "
      f"({'OK' if abs(TARGET_ROLL_DEG) <= _SAT_LIMIT else 'WARNING: near/over saturation'})")
print("  → PARAM_SET PFA_DES_ROLL")
print("  → position setpoint (z=-1 m) keeps pfa_pos_control computing thrust")

current_roll_cmd = 0.0
last_print       = 0.0

while abs(current_roll_cmd - TARGET_ROLL_DEG) > 0.01:
    step      = math.copysign(PARAM_STEP_DEG, TARGET_ROLL_DEG - current_roll_cmd)
    next_roll = current_roll_cmd + step
    if (step > 0 and next_roll > TARGET_ROLL_DEG) or \
       (step < 0 and next_roll < TARGET_ROLL_DEG):
        next_roll = TARGET_ROLL_DEG

    print(f"  PARAM_SET PFA_DES_ROLL = {next_roll:.1f}° ... ", end='', flush=True)
    print("OK" if param_set('PFA_DES_ROLL', next_roll) else "FAILED")
    current_roll_cmd = next_roll

    step_end = time.time() + PARAM_STEP_WAIT
    while time.time() < step_end:
        set_position_target(0.0, 0.0, -TARGET_ALT_M)
        _, _, pz, _, _, _ = get_local_position()
        roll, _, _ = get_attitude()
        alt   = -pz if pz is not None else 0.0
        r_deg = math.degrees(roll) if roll is not None else 0.0
        now   = time.time()
        if now - last_print > 0.5:
            print(f"    alt={alt:.2f} m  roll_cmd={current_roll_cmd:.1f}°  roll_now={r_deg:.1f}°")
            last_print = now
        time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 6 – 追蹤保持
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 6] Holding roll={TARGET_ROLL_DEG:.0f}°, "
      f"alt={TARGET_ALT_M} m for {TRACK_PHASE_DUR:.0f} s...")
print(f"  {'Time':>6}  {'Alt':>6}  {'AltErr':>7}  {'RollCmd':>8}  {'RollNow':>8}")
print("  " + "─" * 44)

track_start = time.time()
last_print  = 0.0

while time.time() - track_start < TRACK_PHASE_DUR:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    _, _, pz, _, _, _ = get_local_position()
    roll, _, _ = get_attitude()
    alt     = -pz if pz is not None else TARGET_ALT_M
    r_deg   = math.degrees(roll) if roll is not None else 0.0
    elapsed = time.time() - track_start
    now     = time.time()
    if now - last_print >= 0.5:
        print(f"  {elapsed:6.1f}s  {alt:6.2f}m  {TARGET_ALT_M-alt:+7.2f}m  "
              f"{TARGET_ROLL_DEG:7.1f}°  {r_deg:7.1f}°")
        last_print = now
    time.sleep(0.05)

print("\n  Roll tracking complete.")

# ─────────────────────────────────────────────────────────────
# Step 7 – 歸零並降落
# ─────────────────────────────────────────────────────────────
print("\n[Step 7] Resetting PFA_DES_ROLL to 0° before landing...")
param_set('PFA_DES_ROLL', 0.0)
time.sleep(1.0)

print("Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
