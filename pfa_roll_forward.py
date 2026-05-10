#!/usr/bin/env python3
"""
PFA Roll Forward (pymavlink)
==============================
Task: 直線前進，roll 從 +45° 到 -45° 來回振盪（正弦波）

軌跡 (NED)：
  x(t) = x₀ + t × PATH_SPEED_MPS   (朝北前進)
  y    = 0  (側向不動)
  z    = -TARGET_ALT_M
  yaw  = 0°

PFA_DES_ROLL(t) = ROLL_AMP × sin(2π t / ROLL_PERIOD_S)
  t=0    →  0°    (水平起步)
  t=T/4  → +45°   (右傾峰值)
  t=T/2  →  0°
  t=3T/4 → -45°   (左傾谷值)
  t=T    →  0°    (一圈完成)

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M    = 1.0    # 懸停高度 (m)
PATH_SPEED_MPS  = 0.5    # 前進速度 (m/s，NED x+)
ROLL_AMP_DEG    = 45.0   # roll 振盪幅度 (deg)；飽和限制 ≈ ±31.6°
ROLL_PERIOD_S   = 6.0    # 每完整振盪週期 (s)
NUM_CYCLES      = 3      # 振盪週期數

HOVER_PHASE_DUR   = 8.0
ALT_TOL           = 0.15
HOVER_STABLE_TIME = 3.0

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
TRACK_DUR_S  = NUM_CYCLES * ROLL_PERIOD_S
TOTAL_PATH_M = PATH_SPEED_MPS * TRACK_DUR_S
OMEGA        = 2.0 * math.pi / ROLL_PERIOD_S

_HOVER_THR  = (1.4 * 9.81) / 24.0
_SAT_LIMIT  = math.degrees(math.asin(0.3 / _HOVER_THR))    # ≈ 31.6°

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}")
print(f"  Roll: ±{ROLL_AMP_DEG:.0f}° × {NUM_CYCLES} cycles  "
      f"({'OK' if ROLL_AMP_DEG <= _SAT_LIMIT else f'WARNING > sat ({_SAT_LIMIT:.1f}°)'})")
print(f"  Path: {PATH_SPEED_MPS} m/s × {TRACK_DUR_S:.0f} s = {TOTAL_PATH_M:.1f} m")


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


def param_set_async(name, value):
    """Fire-and-forget; no ACK wait — safe for 10 Hz oscillation."""
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def set_position_target(x, y, z, yaw_rad=0.0):
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


def wait_for_mode(mode, timeout=5):
    t = time.time()
    while time.time() - t < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and ((msg.custom_mode >> 16) & 0xFF) == mode:
            return True
    return False


def set_mode_offboard():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        6.0, 0, 0, 0, 0, 0)


def set_mode_land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        4.0, 6.0, 0, 0, 0, 0, 0)


def arm(force=False):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0 if force else 0.0, 0, 0, 0, 0, 0)


# ─────────────────────────────────────────────────────────────
# Step 0
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
x, y, z, _, _, _ = get_local_position()
if x is None:
    print("ERROR: No LOCAL_POSITION_NED.")
    sys.exit(1)
print(f"  NED: ({x:.2f}, {y:.2f}, {z:.2f})")

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
    print("ERROR: Failed to arm.")
    sys.exit(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stable hover
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Stable hover at {TARGET_ALT_M} m...")
phase_start  = time.time()
stable_since = None
last_print   = 0.0

while True:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    _, _, pz, _, _, _ = get_local_position()
    if pz is not None:
        alt = -pz
        alt_err = TARGET_ALT_M - alt
        stable_since = (stable_since or time.time()) if abs(alt_err) < ALT_TOL else None
        now = time.time()
        if now - last_print > 1.0:
            s = now - stable_since if stable_since else 0.0
            print(f"  alt={alt:.2f} m  stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
            last_print = now
        if stable_since and (time.time() - stable_since) >= HOVER_STABLE_TIME:
            print(f"  Stable at {alt:.2f} m.")
            break
        if time.time() - phase_start > 25.0:
            print("  Timeout – continuing.")
            break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 3 – Level hover hold
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
end_t = time.time() + HOVER_PHASE_DUR
while time.time() < end_t:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)

px0, _, _, _, _, _ = get_local_position()
path_x0 = px0 if px0 is not None else 0.0
print(f"  Path start x = {path_x0:.2f} m")

# ─────────────────────────────────────────────────────────────
# Step 4 – Forward + roll oscillation
#   x   : x₀ + t × PATH_SPEED_MPS  (直線前進)
#   roll: ROLL_AMP × sin(ωt)        (+45° → -45° → +45° ...)
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] Forward + roll ±{ROLL_AMP_DEG:.0f}°: "
      f"{NUM_CYCLES} cycles × {ROLL_PERIOD_S:.0f} s = {TRACK_DUR_S:.0f} s  "
      f"({TOTAL_PATH_M:.1f} m)")
print(f"  XY saturation ≈ ±{_SAT_LIMIT:.1f}°  "
      f"({'OK' if ROLL_AMP_DEG <= _SAT_LIMIT else 'WARNING: may saturate'})")
print(f"  {'Time':>6}  {'Xcmd':>6}  {'Xnow':>6}  {'Alt':>5}  "
      f"{'RollCmd':>8}  {'RollNow':>8}")
print("  " + "─" * 54)

track_start    = time.time()
last_param_upd = 0.0
last_print     = 0.0

while True:
    now     = time.time()
    elapsed = now - track_start
    if elapsed >= TRACK_DUR_S:
        elapsed = TRACK_DUR_S

    x_cmd    = path_x0 + elapsed * PATH_SPEED_MPS
    roll_cmd = ROLL_AMP_DEG * math.sin(OMEGA * elapsed)

    set_position_target(x_cmd, 0.0, -TARGET_ALT_M, yaw_rad=0.0)

    if now - last_param_upd >= 0.1:
        param_set_async('PFA_DES_ROLL', roll_cmd)
        last_param_upd = now

    if now - last_print >= 0.5:
        px, _, pz, _, _, _ = get_local_position()
        roll, _, _         = get_attitude()
        x_now  = px if px is not None else x_cmd
        alt    = -pz if pz is not None else TARGET_ALT_M
        r_now  = math.degrees(roll) if roll is not None else 0.0
        print(f"  {elapsed:6.1f}s  {x_cmd:6.2f}m  {x_now:6.2f}m  "
              f"{alt:5.2f}m  {roll_cmd:8.1f}°  {r_now:8.1f}°")
        last_print = now

    if elapsed >= TRACK_DUR_S:
        break
    time.sleep(0.05)

print("\n  Roll forward complete.")

# ─────────────────────────────────────────────────────────────
# Step 5 – Reset PFA_DES_ROLL + hold
# ─────────────────────────────────────────────────────────────
print("\n[Step 5] Resetting PFA_DES_ROLL = 0°...")
param_set('PFA_DES_ROLL', 0.0)

px_end, _, _, _, _, _ = get_local_position()
x_end = px_end if px_end is not None else path_x0 + TOTAL_PATH_M
end_t = time.time() + 5.0
while time.time() < end_t:
    set_position_target(x_end, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 6 – Land
# ─────────────────────────────────────────────────────────────
print("\n[Step 6] Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
