#!/usr/bin/env python3
"""
PFA Ramp Mission (pymavlink)
=============================
Task: 往前爬坡起飛 → 巡航 → 斜坡前進下降 → AUTO.LAND

軌跡 (NED，yaw=0° 朝北)：

  Phase 1 – 爬坡起飛
    x(t) = x₀ + t × PATH_SPEED_MPS    (朝北前進)
    z(t) = -(z₀_alt + t × CLIMB_SPEED_MPS)  (高度遞增)  → 至 TARGET_ALT_M 止

  Phase 2 – 水平巡航
    x(t) = x_c + t × PATH_SPEED_MPS
    z    = -TARGET_ALT_M
    持續 CRUISE_DIST_M ÷ PATH_SPEED_MPS 秒

  Phase 3 – 斜坡前進下降
    x(t) = x_d + t × PATH_SPEED_MPS
    z(t) = -TARGET_ALT_M + t × DESCENT_SPEED_MPS  (高度遞減)  → 至 LAND_ALT_M 止

  Phase 4 – AUTO.LAND

PFA_DES_ROLL = PFA_DES_PITCH = 0° 全程；位置控制器自動產生
所需推力，跟隨斜坡位置指令。

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M          = 30.0   # 巡航高度 (m)
CRUISE_DIST_M         = 100.0  # 水平巡航距離 (m)
PATH_SPEED_MPS        = 2.0    # 水平前進速度 (m/s, NED x+)
CLIMB_SPEED_MPS       = 1.0    # 爬升速率 (m/s)
DESCENT_SPEED_MPS     = 1.0    # 下降速率 (m/s)

LIFTOFF_CONFIRM_ALT_M = 1.5    # 確認已離地的高度門檻
LAND_ALT_M            = 3.0    # 切換 AUTO.LAND 的高度門檻

# ─────────────────────────────────────────────────────────────
# Derived (mission summary)
# ─────────────────────────────────────────────────────────────
_CLIMB_DUR_S    = TARGET_ALT_M / CLIMB_SPEED_MPS
_CLIMB_DIST_M   = PATH_SPEED_MPS * _CLIMB_DUR_S
_CRUISE_DUR_S   = CRUISE_DIST_M / PATH_SPEED_MPS
_DESCENT_DUR_S  = (TARGET_ALT_M - LAND_ALT_M) / DESCENT_SPEED_MPS
_DESCENT_DIST_M = PATH_SPEED_MPS * _DESCENT_DUR_S
_TOTAL_DIST_M   = _CLIMB_DIST_M + CRUISE_DIST_M + _DESCENT_DIST_M
_TOTAL_DUR_S    = _CLIMB_DUR_S + _CRUISE_DUR_S + _DESCENT_DUR_S

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}")
print(f"  Mission plan:")
print(f"    Cruise alt    : {TARGET_ALT_M:.0f} m")
print(f"    Cruise dist   : {CRUISE_DIST_M:.0f} m  ({_CRUISE_DUR_S:.0f} s)")
print(f"    Climb  slope  : {PATH_SPEED_MPS:.1f} m/s horiz + {CLIMB_SPEED_MPS:.1f} m/s vert  "
      f"≈ {_CLIMB_DIST_M:.0f} m horiz / {_CLIMB_DUR_S:.0f} s")
print(f"    Descent slope : {PATH_SPEED_MPS:.1f} m/s horiz + {DESCENT_SPEED_MPS:.1f} m/s vert  "
      f"≈ {_DESCENT_DIST_M:.0f} m horiz / {_DESCENT_DUR_S:.0f} s")
print(f"    Total horiz   : ≈ {_TOTAL_DIST_M:.0f} m  ({_TOTAL_DUR_S:.0f} s)")


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


def get_altitude():
    _, _, pz, _, _, _ = get_local_position()
    return -pz if pz is not None else None


def get_x():
    px, _, _, _, _, _ = get_local_position()
    return px


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

# Stream low setpoint for OFFBOARD precondition
TAKEOFF_ALT_M = 2.0
print(f"  Streaming setpoints (5 s, z={-TAKEOFF_ALT_M:.1f})...")
for _ in range(100):
    set_position_target(x0, y0, -TAKEOFF_ALT_M)
    time.sleep(0.05)

set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD confirmed.")
else:
    for _ in range(50):
        set_position_target(x0, y0, -TAKEOFF_ALT_M)
        time.sleep(0.05)
    set_mode_offboard()
    print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: not confirmed.")

for _ in range(20):
    set_position_target(x0, y0, -TAKEOFF_ALT_M)
    time.sleep(0.05)
arm()

armed = False
for _ in range(100):
    set_position_target(x0, y0, -TAKEOFF_ALT_M)
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    arm(force=True)
    for _ in range(100):
        set_position_target(x0, y0, -TAKEOFF_ALT_M)
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
# Step 2 – Wait for liftoff confirmation
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Waiting for liftoff (alt > {LIFTOFF_CONFIRM_ALT_M:.1f} m)...")
last_print = 0.0
t_phase    = time.time()

while True:
    set_position_target(x0, y0, -TAKEOFF_ALT_M)
    alt = get_altitude()
    now = time.time()
    if alt is not None and now - last_print > 1.0:
        print(f"  alt={alt:.2f} m")
        last_print = now
    if alt is not None and alt >= LIFTOFF_CONFIRM_ALT_M:
        print(f"  Liftoff confirmed at alt={alt:.2f} m.")
        break
    if now - t_phase > 20.0:
        print("  Liftoff timeout – continuing anyway.")
        break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 3 – Phase 1: Climb ramp (North + ascent)
# ─────────────────────────────────────────────────────────────
x0_climb   = x0
alt0_climb = get_altitude() or LIFTOFF_CONFIRM_ALT_M

print(f"\n[Step 3] CLIMB RAMP: ({x0_climb:.1f} m North, alt {alt0_climb:.1f} m) "
      f"→ alt={TARGET_ALT_M:.0f} m")
print(f"  Speed: horiz {PATH_SPEED_MPS:.1f} m/s + vert {CLIMB_SPEED_MPS:.1f} m/s")
print(f"  {'Time':>6}  {'Xcmd':>6}  {'Xnow':>6}  {'Altcmd':>7}  {'Altnow':>7}")
print("  " + "─" * 42)

t_phase    = time.time()
last_print = 0.0
x_cmd      = x0_climb

while True:
    now     = time.time()
    elapsed = now - t_phase

    x_cmd   = x0_climb   + elapsed * PATH_SPEED_MPS
    alt_cmd = alt0_climb  + elapsed * CLIMB_SPEED_MPS
    if alt_cmd >= TARGET_ALT_M:
        alt_cmd = TARGET_ALT_M

    set_position_target(x_cmd, y0, -alt_cmd, yaw_rad=0.0)

    alt_now = get_altitude()
    if now - last_print > 1.0:
        print(f"  {elapsed:6.1f}s  {x_cmd:6.1f}m  {get_x() or 0.0:6.1f}m  "
              f"{alt_cmd:7.1f}m  {alt_now or 0.0:7.1f}m")
        last_print = now

    if alt_cmd >= TARGET_ALT_M:
        break
    time.sleep(0.05)

x_cruise_start = x_cmd
print(f"\n  Climb complete. x ≈ {x_cruise_start:.1f} m  alt_cmd = {TARGET_ALT_M:.0f} m")

# ─────────────────────────────────────────────────────────────
# Step 4 – Phase 2: Level cruise (North)
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] CRUISE: alt={TARGET_ALT_M:.0f} m  →  {CRUISE_DIST_M:.0f} m North "
      f"({_CRUISE_DUR_S:.0f} s)")
print(f"  {'Time':>6}  {'Xcmd':>7}  {'Xnow':>7}  {'Altnow':>7}  {'Remaining':>10}")
print("  " + "─" * 48)

t_phase    = time.time()
last_print = 0.0

while True:
    now     = time.time()
    elapsed = now - t_phase

    x_cmd = x_cruise_start + elapsed * PATH_SPEED_MPS
    set_position_target(x_cmd, y0, -TARGET_ALT_M, yaw_rad=0.0)

    alt_now = get_altitude()
    if now - last_print > 1.0:
        remaining = CRUISE_DIST_M - (x_cmd - x_cruise_start)
        print(f"  {elapsed:6.1f}s  {x_cmd:7.1f}m  {get_x() or 0.0:7.1f}m  "
              f"{alt_now or 0.0:7.1f}m  {max(0.0, remaining):8.1f}m")
        last_print = now

    if x_cmd - x_cruise_start >= CRUISE_DIST_M:
        break
    time.sleep(0.05)

x_descent_start = x_cmd
print(f"\n  Cruise complete. x ≈ {x_descent_start:.1f} m")

# ─────────────────────────────────────────────────────────────
# Step 5 – Phase 3: Descent ramp (North + descent)
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 5] DESCENT RAMP: alt={TARGET_ALT_M:.0f} m → {LAND_ALT_M:.0f} m "
      f"while flying North")
print(f"  Speed: horiz {PATH_SPEED_MPS:.1f} m/s + vert {DESCENT_SPEED_MPS:.1f} m/s")
print(f"  {'Time':>6}  {'Xcmd':>7}  {'Xnow':>7}  {'Altcmd':>7}  {'Altnow':>7}")
print("  " + "─" * 42)

t_phase    = time.time()
last_print = 0.0

while True:
    now     = time.time()
    elapsed = now - t_phase

    x_cmd   = x_descent_start + elapsed * PATH_SPEED_MPS
    alt_cmd = TARGET_ALT_M   - elapsed * DESCENT_SPEED_MPS
    if alt_cmd <= LAND_ALT_M:
        alt_cmd = LAND_ALT_M

    set_position_target(x_cmd, y0, -alt_cmd, yaw_rad=0.0)

    alt_now = get_altitude()
    if now - last_print > 1.0:
        print(f"  {elapsed:6.1f}s  {x_cmd:7.1f}m  {get_x() or 0.0:7.1f}m  "
              f"{alt_cmd:7.1f}m  {alt_now or 0.0:7.1f}m")
        last_print = now

    if alt_cmd <= LAND_ALT_M:
        break
    time.sleep(0.05)

alt_final = get_altitude()
x_final   = get_x()
print(f"\n  Descent ramp complete. x ≈ {x_final or 0.0:.1f} m  alt ≈ {alt_final or 0.0:.1f} m")

# ─────────────────────────────────────────────────────────────
# Step 6 – Land
# ─────────────────────────────────────────────────────────────
print("\n[Step 6] Switching to AUTO.LAND...")
set_mode_land()
print("  Waiting for touchdown (20 s)...")
time.sleep(20)
print("Done.")
