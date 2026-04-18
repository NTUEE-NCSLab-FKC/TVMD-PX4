#!/usr/bin/env python3
"""
PFA Sin Wave Path Tracking (pymavlink)
========================================
Task: 追蹤水平正弦波路徑，roll 與 pitch 維持 0 度

軌跡 (NED，俯視圖)：
  x(t) = x₀ + t × PATH_SPEED_MPS           (朝北前進)
  y(t) = y₀ + SIN_AMPLITUDE_M × sin(2π t / SIN_PERIOD_S)  (東西振盪)
  z    = -TARGET_ALT_M                       (高度保持)
  yaw  = 0°                                  (始終朝北)

空間波長：λ = PATH_SPEED_MPS × SIN_PERIOD_S   (m/cycle)
峰值側向速度：Vy_max = SIN_AMPLITUDE_M × 2π / SIN_PERIOD_S  (m/s)

控制架構：
  Python → SET_POSITION_TARGET_LOCAL_NED (x=x(t), y=y(t), z=-1, yaw=0)
  PX4:  pfa_pos_control → thrust (追蹤 sin 波位置)
                        → attitude_des = (0, 0, safe_yaw)   ← PFA_DES_ROLL/PITCH=0
        pfa_att_control → roll=0°, pitch=0°  (PFA 位置/姿態解耦)

PFA 特性展示：位置沿正弦波運動，機體保持水平，不受路徑曲率影響。

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 1.0    # 懸停高度 (m)
PATH_SPEED_MPS    = 0.5    # 前進速度 (m/s, NED x+)
SIN_AMPLITUDE_M   = 1.0    # 正弦波幅度 (m, NED y 方向，東西振盪)
SIN_PERIOD_S      = 4.0    # 振盪週期 (s/cycle)
NUM_CYCLES        = 4      # 完整週期數

HOVER_PHASE_DUR   = 8.0
ALT_TOL           = 0.15
HOVER_STABLE_TIME = 3.0

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
TRACK_DUR_S      = NUM_CYCLES * SIN_PERIOD_S
TOTAL_PATH_M     = PATH_SPEED_MPS * TRACK_DUR_S
SPATIAL_LAMBDA_M = PATH_SPEED_MPS * SIN_PERIOD_S
PEAK_VY_MPS      = SIN_AMPLITUDE_M * 2.0 * math.pi / SIN_PERIOD_S

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}")
print(f"  x: {PATH_SPEED_MPS} m/s × {TRACK_DUR_S:.0f} s = {TOTAL_PATH_M:.1f} m forward (NED North)")
print(f"  y: ±{SIN_AMPLITUDE_M:.1f} m sin wave, T={SIN_PERIOD_S:.0f} s, λ={SPATIAL_LAMBDA_M:.1f} m")
print(f"  Vy_max = {PEAK_VY_MPS:.2f} m/s,  cycles = {NUM_CYCLES}")
print(f"  Attitude: roll=0°, pitch=0° (PFA 位置/姿態解耦)")


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


# ─────────────────────────────────────────────────────────────
# Step 0 – 確保 PFA_DES_ROLL/PITCH = 0°（水平飛行）
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0° (level flight)...")
print("  PFA_DES_ROLL  = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED")
print("  PFA_DES_PITCH = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED")

# ─────────────────────────────────────────────────────────────
# Step 1 – Pre-flight
# ─────────────────────────────────────────────────────────────
print("\n[Step 1] Pre-flight check...")
x, y, z, _, _, _ = get_local_position()
if x is None:
    print("ERROR: No LOCAL_POSITION_NED.")
    sys.exit(1)
print(f"  NED position: ({x:.2f}, {y:.2f}, {z:.2f})")

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
print(f"\n[Step 3] Stable hover at {TARGET_ALT_M} m...")
print(f"  Waiting (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

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
# Step 4 – Level hover hold
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
end_t      = time.time() + HOVER_PHASE_DUR
last_print = 0.0
while time.time() < end_t:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        _, _, pz, _, _, _ = get_local_position()
        alt = -pz if pz is not None else TARGET_ALT_M
        print(f"  alt={alt:.2f} m  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# Record origin
px0, py0, _, _, _, _ = get_local_position()
path_x0 = px0 if px0 is not None else 0.0
path_y0 = py0 if py0 is not None else 0.0
print(f"  Path origin: NED ({path_x0:.2f}, {path_y0:.2f}) m")

# ─────────────────────────────────────────────────────────────
# Step 5 – Sin wave path tracking
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 5] Sin wave path: {NUM_CYCLES} cycles × {SIN_PERIOD_S:.0f} s = {TRACK_DUR_S:.0f} s")
print(f"  {'Time':>6}  {'Xcmd':>6}  {'Xnow':>6}  {'Ycmd':>6}  {'Ynow':>6}  "
      f"{'Alt':>5}  {'Roll':>7}  {'Pitch':>7}")
print("  " + "─" * 66)

track_start = time.time()
last_print  = 0.0

while True:
    now     = time.time()
    elapsed = now - track_start

    if elapsed >= TRACK_DUR_S:
        elapsed = TRACK_DUR_S

    x_cmd = path_x0 + elapsed * PATH_SPEED_MPS
    y_cmd = path_y0 + SIN_AMPLITUDE_M * math.sin(
        2.0 * math.pi * elapsed / SIN_PERIOD_S)

    set_position_target(x_cmd, y_cmd, -TARGET_ALT_M, yaw_rad=0.0)

    if now - last_print >= 0.5:
        px, py, pz, _, _, _ = get_local_position()
        roll, pitch, _      = get_attitude()
        x_now = px if px is not None else x_cmd
        y_now = py if py is not None else y_cmd
        alt   = -pz if pz is not None else TARGET_ALT_M
        r_deg = math.degrees(roll)  if roll  is not None else 0.0
        p_deg = math.degrees(pitch) if pitch is not None else 0.0
        print(f"  {elapsed:6.1f}s  {x_cmd:6.2f}m  {x_now:6.2f}m  "
              f"{y_cmd:6.2f}m  {y_now:6.2f}m  {alt:5.2f}m  "
              f"{r_deg:6.1f}°  {p_deg:6.1f}°")
        last_print = now

    if elapsed >= TRACK_DUR_S:
        break
    time.sleep(0.05)

print("\n  Sin path complete.")

# ─────────────────────────────────────────────────────────────
# Step 6 – Hold end position for 10 s
# ─────────────────────────────────────────────────────────────
px_end, py_end, _, _, _, _ = get_local_position()
x_end = px_end if px_end is not None else path_x0 + TOTAL_PATH_M
y_end = py_end if py_end is not None else path_y0

print(f"\n[Step 6] Holding end position ({x_end:.2f}, {y_end:.2f}) m for 10 s...")
end_t      = time.time() + 10.0
last_print = 0.0
while time.time() < end_t:
    set_position_target(x_end, y_end, -TARGET_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        px, py, pz, _, _, _ = get_local_position()
        roll, pitch, _ = get_attitude()
        alt   = -pz if pz is not None else TARGET_ALT_M
        r_deg = math.degrees(roll)  if roll  is not None else 0.0
        p_deg = math.degrees(pitch) if pitch is not None else 0.0
        print(f"  x={px:.2f} y={py:.2f} alt={alt:.2f} m  "
              f"roll={r_deg:.1f}° pitch={p_deg:.1f}°  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 7 – Land
# ─────────────────────────────────────────────────────────────
print("\n[Step 7] Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
