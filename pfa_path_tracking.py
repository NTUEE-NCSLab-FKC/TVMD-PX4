#!/usr/bin/env python3
"""
PFA Path Tracking (pymavlink)
==============================
Task: 直線前進，先 roll ±ROLL_AMPLITUDE° × N 週期，再 pitch ±PITCH_AMPLITUDE° × N 週期

軌跡 (NED)：
  x = x_start + t × PATH_SPEED_MPS   (朝北前進)
  y = 0                               (側向保持)
  z = -TARGET_ALT_M                   (高度保持)
  yaw = 0°                            (朝北)

振盪波形：A × sin(2π t / T)
  t=0   → 0°        (起始水平)
  t=T/4 → +A°       (peak: roll 右傾 / pitch 仰頭)
  t=T/2 → 0°        (回水平)
  t=3T/4 → -A°      (trough: roll 左傾 / pitch 低頭)
  t=T   → 0°        (一個完整週期)

控制架構：
  Python → SET_POSITION_TARGET_LOCAL_NED (x=t×v, y=0, z=-1, yaw=0)
         → PARAM_SET PFA_DES_ROLL / PFA_DES_PITCH = A × sin(2π t / T)  [async 10 Hz]
  PX4:  pfa_pos_control → thrust (追蹤 x_cmd 位置)
        pfa_att_control → roll / pitch (追蹤振盪波形)

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M         = 1.0    # 懸停高度 (m)
PATH_SPEED_MPS       = 0.5    # 前進速度 (m/s, NED x+)

ROLL_AMPLITUDE_DEG   = 45.0   # roll 振盪幅度 (deg)；飽和限制 ≈ ±31.6°
PITCH_AMPLITUDE_DEG  = 45.0   # pitch 振盪幅度 (deg)
OSCILLATION_PERIOD_S = 6.0    # 振盪週期 (s/cycle)
NUM_ROLL_CYCLES      = 2      # roll 完整振盪次數
NUM_PITCH_CYCLES     = 2      # pitch 完整振盪次數

HOVER_PHASE_DUR      = 8.0    # 穩定懸停後等待時間 (s)
ALT_TOL              = 0.15   # 高度容忍範圍 (m)
HOVER_STABLE_TIME    = 3.0    # 高度穩定判定時間 (s)

# ─────────────────────────────────────────────────────────────
# Derived constants
# ─────────────────────────────────────────────────────────────
ROLL_PHASE_DUR  = NUM_ROLL_CYCLES  * OSCILLATION_PERIOD_S   # 12 s
PITCH_PHASE_DUR = NUM_PITCH_CYCLES * OSCILLATION_PERIOD_S   # 12 s
TOTAL_PATH_M    = PATH_SPEED_MPS * (ROLL_PHASE_DUR + PITCH_PHASE_DUR)  # 12 m

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
print(f"  Roll:  ±{ROLL_AMPLITUDE_DEG:.0f}°  "
      f"{'OK' if ROLL_AMPLITUDE_DEG  <= _SAT_LIMIT else f'WARNING: > saturation ({_SAT_LIMIT:.1f}°)'}")
print(f"  Pitch: ±{PITCH_AMPLITUDE_DEG:.0f}°  "
      f"{'OK' if PITCH_AMPLITUDE_DEG <= _SAT_LIMIT else f'WARNING: > saturation ({_SAT_LIMIT:.1f}°)'}")
print(f"  Path:  {PATH_SPEED_MPS} m/s × {ROLL_PHASE_DUR+PITCH_PHASE_DUR:.0f} s"
      f" = {TOTAL_PATH_M:.1f} m (NED x+, North)")


# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────

def param_set(name, value, retries=5):
    """Blocking param set with ACK; used for init/cleanup."""
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
    """Fire-and-forget param set (no ACK wait).
    Safe for 10 Hz oscillation updates; doesn't block the main loop."""
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


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
# Core oscillation loop
# ─────────────────────────────────────────────────────────────

def run_oscillation_phase(param_name, amplitude_deg, phase_dur, path_start_x):
    """
    直線前進同時對 param_name 施加正弦振盪。

    param_name    : 'PFA_DES_ROLL' 或 'PFA_DES_PITCH'
    amplitude_deg : 振盪幅度 (deg)
    phase_dur     : 本階段持續時間 (s)
    path_start_x  : 本階段起始 NED x 座標 (m)

    位置 setpoint 每 50 ms 發送一次（20 Hz）；
    param 每 100 ms async 更新一次（10 Hz，無 ACK 阻塞）。

    返回：本階段結束時的實際 x 座標。
    """
    label = 'roll' if 'ROLL' in param_name else 'pitch'
    phase_start    = time.time()
    last_param_upd = 0.0
    last_print     = 0.0

    print(f"  {'Time':>6}  {'X_cmd':>6}  {'X_now':>6}  {'Alt':>5}  "
          f"{label+'_cmd':>10}  {label+'_now':>10}")
    print("  " + "─" * 56)

    while True:
        now     = time.time()
        elapsed = now - phase_start
        if elapsed >= phase_dur:
            elapsed = phase_dur

        # ── position command (straight-line forward) ──────────
        x_cmd = path_start_x + elapsed * PATH_SPEED_MPS
        set_position_target(x_cmd, 0.0, -TARGET_ALT_M, yaw_rad=0.0)

        # ── attitude command (sinusoidal) ─────────────────────
        angle_cmd = amplitude_deg * math.sin(
            2.0 * math.pi * elapsed / OSCILLATION_PERIOD_S)

        # update param at 10 Hz; async so it never blocks the loop
        if now - last_param_upd >= 0.1:
            param_set_async(param_name, angle_cmd)
            last_param_upd = now

        # ── logging at 2 Hz ───────────────────────────────────
        if now - last_print >= 0.5:
            px, _, pz, _, _, _ = get_local_position()
            roll, pitch, _     = get_attitude()
            alt   = -pz if pz is not None else TARGET_ALT_M
            x_now = px  if px is not None else x_cmd
            fb_rad = roll if 'ROLL' in param_name else pitch
            fb_deg = math.degrees(fb_rad) if fb_rad is not None else 0.0
            print(f"  {elapsed:6.1f}s  {x_cmd:6.2f}m  {x_now:6.2f}m  "
                  f"{alt:5.2f}m  {angle_cmd:9.1f}°  {fb_deg:9.1f}°")
            last_print = now

        if elapsed >= phase_dur:
            break
        time.sleep(0.05)

    # return actual x for smooth handoff to the next phase
    px, _, _, _, _, _ = get_local_position()
    return px if px is not None else path_start_x + phase_dur * PATH_SPEED_MPS


# ─────────────────────────────────────────────────────────────
# Step 0 – 起飛前將 PFA_DES_ROLL / PFA_DES_PITCH 歸零
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Initializing PFA attitude parameters...")
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
# Step 3 – 位置控制懸停至 TARGET_ALT_M
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Hover at {TARGET_ALT_M} m (PFA_DES_ROLL/PITCH = 0°)...")
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
# Step 4 – Level hover hold (HOVER_PHASE_DUR s)
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] Level hover hold for {HOVER_PHASE_DUR:.0f} s...")
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

# Record x at path start
px0, _, _, _, _, _ = get_local_position()
path_start_x = px0 if px0 is not None else 0.0
print(f"  Path start x = {path_start_x:.2f} m (NED North)")

# ─────────────────────────────────────────────────────────────
# Step 5 – Roll oscillation + straight-line forward
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 5] Roll oscillation: ±{ROLL_AMPLITUDE_DEG:.0f}° × {NUM_ROLL_CYCLES} cycles  "
      f"[{ROLL_PHASE_DUR:.0f} s, Δx = {PATH_SPEED_MPS*ROLL_PHASE_DUR:.1f} m]")
print(f"  XY saturation limit ≈ ±{_SAT_LIMIT:.1f}°  "
      f"({'OK' if ROLL_AMPLITUDE_DEG <= _SAT_LIMIT else 'WARNING: may saturate XY thrust'})")
print(f"  PFA_DES_ROLL = {ROLL_AMPLITUDE_DEG:.0f} × sin(2π t / {OSCILLATION_PERIOD_S:.0f} s)")

current_x = run_oscillation_phase(
    'PFA_DES_ROLL', ROLL_AMPLITUDE_DEG, ROLL_PHASE_DUR, path_start_x)

print(f"\n  Roll phase done. x = {current_x:.2f} m")
print("  Resetting PFA_DES_ROLL = 0° ...")
param_set('PFA_DES_ROLL', 0.0)
# brief level pause between phases
pause_end = time.time() + 2.0
while time.time() < pause_end:
    set_position_target(current_x, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 6 – Pitch oscillation + continue straight-line forward
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 6] Pitch oscillation: ±{PITCH_AMPLITUDE_DEG:.0f}° × {NUM_PITCH_CYCLES} cycles  "
      f"[{PITCH_PHASE_DUR:.0f} s, Δx = {PATH_SPEED_MPS*PITCH_PHASE_DUR:.1f} m]")
print(f"  XY saturation limit ≈ ±{_SAT_LIMIT:.1f}°  "
      f"({'OK' if PITCH_AMPLITUDE_DEG <= _SAT_LIMIT else 'WARNING: may saturate XY thrust'})")
print(f"  PFA_DES_PITCH = {PITCH_AMPLITUDE_DEG:.0f} × sin(2π t / {OSCILLATION_PERIOD_S:.0f} s)")

current_x = run_oscillation_phase(
    'PFA_DES_PITCH', PITCH_AMPLITUDE_DEG, PITCH_PHASE_DUR, current_x)

print(f"\n  Pitch phase done. x = {current_x:.2f} m")
print("  Resetting PFA_DES_PITCH = 0° ...")
param_set('PFA_DES_PITCH', 0.0)

# ─────────────────────────────────────────────────────────────
# Step 7 – Hold end position for 10 s
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 7] Holding end position x ≈ {current_x:.1f} m for 10 s...")
end_t      = time.time() + 10.0
last_print = 0.0
while time.time() < end_t:
    set_position_target(current_x, 0.0, -TARGET_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        px, _, pz, _, _, _ = get_local_position()
        roll, pitch, _     = get_attitude()
        alt   = -pz if pz is not None else TARGET_ALT_M
        x_now = px  if px is not None else current_x
        r_deg = math.degrees(roll)  if roll  is not None else 0.0
        p_deg = math.degrees(pitch) if pitch is not None else 0.0
        print(f"  x={x_now:.2f} m  alt={alt:.2f} m  "
              f"roll={r_deg:.1f}°  pitch={p_deg:.1f}°  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 8 – Land
# ─────────────────────────────────────────────────────────────
print("\n[Step 8] Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
