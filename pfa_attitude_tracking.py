#!/usr/bin/env python3
"""
PFA Attitude Tracking Task (pymavlink)
=======================================
Hover at altitude 1 m with pitch maintained at 45 degrees.

Control architecture
--------------------
pfa_pos_control 的職責分工：

  ┌─────────────────────────────────────────────────────┐
  │              pose_controller_6dof()                  │
  │                                                      │
  │  INPUT  trajectory_setpoint  ──► PD position ctrl   │
  │         (x=0, y=0, z=-1 m)       err_pos, err_vel   │
  │                                  + resist_gravity    │
  │                                  → control_acc (NED) │
  │                                  → thrust_body  ◄── pfa_pos_control 計算 │
  │                                                      │
  │  INPUT  euler_attitude_des ──────────────────────►  │
  │         (roll=0, pitch=45°, yaw=0)                  │  pass-through
  │         ← 由 PFA_DES_PITCH 參數提供                 │
  │                                                      │
  │  OUTPUT vehicle_attitude_setpoint                    │
  │           .thrust_body   ← pfa_pos_control 計算     │
  │           .roll/pitch/yaw_body ← 參數輸入 pass-through│
  └─────────────────────────────────────────────────────┘

修改摘要 (pfa_pos_control.cpp)：
  原本：attitude_des = Vector3f(0.0f, 0.0f, traj.yaw)
  現在：attitude_des = Vector3f(
            radians(PFA_DES_ROLL),   ← 新增參數，預設 0°
            radians(PFA_DES_PITCH),  ← 新增參數，預設 0°
            traj.yaw)

Python 端使用 MAVLink PARAM_SET 在飛行前設定 PFA_DES_PITCH=45，
其餘控制全部交給 pfa_pos_control。

Connection: udp:127.0.0.1:14550  (PX4 SITL)
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M     = 1.0          # 懸停高度 (m)
TARGET_PITCH_DEG = 45.0         # 期望 pitch (deg, 正值 = nose up)
TARGET_ROLL_DEG  = 0.0          # 期望 roll  (deg)

HOVER_PHASE_DUR  = 8.0          # 穩定懸停後 pitch ramp 前的等待時間 (s)
PITCH_RAMP_TIME  = 4.0          # pitch 從 0° 爬升到 45° 所需秒數 (漸進避免突變)
TRACK_PHASE_DUR  = 30.0         # 維持 pitch=45° 的追蹤時長 (s)

# 達到高度的容忍度
ALT_TOL          = 0.15         # m
HOVER_STABLE_TIME = 3.0         # 在容忍範圍內持續 N 秒才確認穩定

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting to PX4 (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")
print(f"  Task: hover at {TARGET_ALT_M} m, pitch = {TARGET_PITCH_DEG}°")


# ─────────────────────────────────────────────────────────────
# MAVLink helpers
# ─────────────────────────────────────────────────────────────

def param_set(name: str, value: float, retries: int = 5) -> bool:
    """
    Set a PX4 parameter via MAVLink PARAM_SET and wait for PARAM_VALUE ACK.
    pfa_pos_control 會在下一個 parameters_update() 週期套用新值。
    """
    name_bytes = name.encode('utf-8')
    for attempt in range(retries):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name_bytes,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        ack = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if ack and ack.param_id.rstrip('\x00') == name:
            return True
        time.sleep(0.3)
    return False


def param_get(name: str):
    """Read a parameter from the flight controller."""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        -1,
    )
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg and msg.param_id.rstrip('\x00') == name:
        return msg.param_value
    return None


def set_position_target(x: float, y: float, z: float):
    """
    SET_POSITION_TARGET_LOCAL_NED — position only, NED frame (z < 0 = up).
    Feeds trajectory_setpoint on PX4, which pfa_pos_control reads.
    type_mask = 0b0000_1111_1111_1000 → use position, ignore vel/acc/yaw/yaw_rate
    """
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )


def get_local_position():
    """(x, y, z, vx, vy, vz) in NED; all None on timeout."""
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg:
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
    return None, None, None, None, None, None


def get_attitude():
    """(roll, pitch, yaw) in radians; all None on timeout."""
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        return msg.roll, msg.pitch, msg.yaw
    return None, None, None


def wait_for_mode(target_main_mode: int, timeout: float = 5) -> bool:
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
        6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )


def set_mode_land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    )


def arm(force: bool = False):
    p2 = 21196.0 if force else 0.0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, p2, 0.0, 0.0, 0.0, 0.0, 0.0,
    )


# ─────────────────────────────────────────────────────────────
# Step 0 – 設定 PFA_DES_PITCH / PFA_DES_ROLL 參數
#           (飛行前設定，pfa_pos_control 會在下一個 parameters_update 生效)
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Configuring PFA attitude target parameters...")

# 先確認連線可以讀到參數
current_pitch_param = param_get('PFA_DES_PITCH')
if current_pitch_param is None:
    print("  WARNING: Could not read PFA_DES_PITCH. Firmware may not have this parameter yet.")
    print("  → Apply the pfa_pos_control.cpp / pfa_pos_control_params.c patch and rebuild.")
else:
    print(f"  PFA_DES_PITCH current = {current_pitch_param:.1f}°")

# 初始設定為 0°（起飛時維持水平）
print("  Setting PFA_DES_ROLL  = 0°  ... ", end='', flush=True)
ok = param_set('PFA_DES_ROLL', 0.0)
print("OK" if ok else "FAILED (continuing)")

print("  Setting PFA_DES_PITCH = 0°  ... ", end='', flush=True)
ok = param_set('PFA_DES_PITCH', 0.0)
print("OK" if ok else "FAILED (continuing)")

# ─────────────────────────────────────────────────────────────
# Step 1 – Pre-flight check
# ─────────────────────────────────────────────────────────────
print("\n[Step 1] Checking local position estimate...")
x, y, z, _, _, _ = get_local_position()
if x is None:
    print("  ERROR: No LOCAL_POSITION_NED. Is EKF2 running?")
    sys.exit(1)
print(f"  OK – NED position: ({x:.2f}, {y:.2f}, {z:.2f})")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Streaming initial setpoints (5 s, z={-TARGET_ALT_M:.1f} NED)...")
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
    print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: not confirmed, continuing.")

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
    print("  ERROR: Failed to arm.")
    sys.exit(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – 位置控制懸停至 1 m（pfa_pos_control, PFA_DES_PITCH=0°）
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Position-control hover at {TARGET_ALT_M} m  (PFA_DES_PITCH=0°)")
print( "  → pfa_pos_control computes thrust from position PD errors")
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
            _, pitch, _ = get_attitude()
            p_deg = math.degrees(pitch) if pitch is not None else 0.0
            s = now - stable_since if stable_since else 0.0
            print(f"  alt={alt:.2f} m (err={alt_err:+.2f})  pitch={p_deg:.1f}°  "
                  f"stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
            last_print = now

        if stable_since and (time.time() - stable_since) >= HOVER_STABLE_TIME:
            print(f"  Stable hover at {alt:.2f} m.")
            break
        if time.time() - phase_start > 25.0:
            print(f"  Hover timeout – current alt={alt:.2f} m, continuing.")
            break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 4 – 保持水平懸停 HOVER_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
end_t = time.time() + HOVER_PHASE_DUR
last_print = 0.0
while time.time() < end_t:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        px, py, pz, _, _, _ = get_local_position()
        _, pitch, _ = get_attitude()
        alt   = -pz if pz is not None else 0.0
        p_deg = math.degrees(pitch) if pitch is not None else 0.0
        print(f"  alt={alt:.2f} m  pitch={p_deg:.1f}°  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 5 – 漸進式 pitch ramp：透過 PARAM_SET 逐步更新 PFA_DES_PITCH
#
#  ┌─ Python ─────────────────────────────────────────────────┐
#  │ PARAM_SET PFA_DES_PITCH = θ(t)  (0° → 45° 線性爬升)     │
#  └──────────────────────┬───────────────────────────────────┘
#                         ▼ parameters_update()
#  ┌─ pfa_pos_control (PX4) ──────────────────────────────────┐
#  │ attitude_des = (radians(PFA_DES_ROLL),                   │
#  │                radians(PFA_DES_PITCH),  ← 讀新參數       │
#  │                traj.yaw)                                  │
#  │                                                          │
#  │ thrust_body = rotateVectorInverse(                       │
#  │     (Kp*err_pos + Kv*err_vel + resist_gravity)*m/F_max,  │
#  │     q_att_current)          ← 用當前姿態轉換，非期望姿態  │
#  │                                                          │
#  │ publish vehicle_attitude_setpoint:                       │
#  │   .thrust_body   ← pfa_pos_control 計算                 │
#  │   .pitch_body    ← PFA_DES_PITCH (pass-through)         │
#  └─────────────────────┬────────────────────────────────────┘
#                        ▼
#  ┌─ pfa_att_control / mc_att_control ───────────────────────┐
#  │  驅動機體達到 pitch = 45°                                 │
#  └──────────────────────────────────────────────────────────┘
# ─────────────────────────────────────────────────────────────
PARAM_STEP_DEG  = 5.0           # 每次 PARAM_SET 的 pitch 步進量 (deg，恆正)
# 用 abs() 確保正負 pitch 都能正確計算等待時間
PARAM_STEP_WAIT = PITCH_RAMP_TIME / (abs(TARGET_PITCH_DEG) / PARAM_STEP_DEG)

# 物理限制提示：_thrust_xy_max = 0.3，飽和條件 |hover_thrust * sin(pitch)| = 0.3
# 飽和角 = arcsin(0.3 / (m*g/F_max)) = arcsin(0.3/0.572) ≈ ±31.6°
# |pitch| < 31.6° → 完整追蹤；|pitch| > 31.6° → XY 板手飽和，位置可能漂移
_sat_limit = math.degrees(math.asin(0.3 / ((1.4 * 9.81) / 24.0)))
print(f"\n[Step 5] Pitch ramp: 0° → {TARGET_PITCH_DEG:.0f}° "
      f"(step={PARAM_STEP_DEG:.0f}°, interval={PARAM_STEP_WAIT:.1f} s/step)")
print(f"  XY thrust saturation limit: ±{_sat_limit:.1f}°  "
      f"({'OK' if abs(TARGET_PITCH_DEG) <= _sat_limit else 'WARNING: near/over saturation'})")
print( "  → PARAM_SET PFA_DES_PITCH  so pfa_pos_control carries the attitude target")
print( "  → position setpoint (z=-1 m) keeps pfa_pos_control computing thrust")

current_pitch_cmd = 0.0
ramp_start = time.time()
last_print = 0.0

# 正負 pitch 均支援：step 方向跟隨 TARGET_PITCH_DEG 的符號，clamp 防止 overshoot
while abs(current_pitch_cmd - TARGET_PITCH_DEG) > 0.01:
    step       = math.copysign(PARAM_STEP_DEG, TARGET_PITCH_DEG - current_pitch_cmd)
    next_pitch = current_pitch_cmd + step
    # clamp：不超過目標
    if (step > 0 and next_pitch > TARGET_PITCH_DEG) or \
       (step < 0 and next_pitch < TARGET_PITCH_DEG):
        next_pitch = TARGET_PITCH_DEG

    print(f"  PARAM_SET PFA_DES_PITCH = {next_pitch:.1f}° ... ", end='', flush=True)
    ok = param_set('PFA_DES_PITCH', next_pitch)
    print("OK" if ok else "FAILED")
    current_pitch_cmd = next_pitch

    # 每一步等待姿態響應，同時持續送位置目標
    step_end = time.time() + PARAM_STEP_WAIT
    while time.time() < step_end:
        set_position_target(0.0, 0.0, -TARGET_ALT_M)

        px, py, pz, vx, vy, vz = get_local_position()
        _, pitch, _ = get_attitude()

        alt   = -pz if pz is not None else 0.0
        p_deg = math.degrees(pitch) if pitch is not None else 0.0

        now = time.time()
        if now - last_print > 0.5:
            print(f"    alt={alt:.2f} m  pitch_cmd={current_pitch_cmd:.1f}°  pitch_now={p_deg:.1f}°")
            last_print = now
        time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 6 – 維持 pitch=45°, alt=1 m，持續追蹤 TRACK_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 6] Holding pitch={TARGET_PITCH_DEG:.0f}°, alt={TARGET_ALT_M} m "
      f"for {TRACK_PHASE_DUR:.0f} s...")
print(f"  {'Time':>6}  {'Alt':>6}  {'AltErr':>7}  {'PitchCmd':>9}  {'PitchNow':>9}")
print("  " + "─" * 46)

track_start = time.time()
last_print  = 0.0

while time.time() - track_start < TRACK_PHASE_DUR:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)

    px, py, pz, _, _, _ = get_local_position()
    _, pitch, _ = get_attitude()

    alt     = -pz if pz is not None else TARGET_ALT_M
    alt_err = TARGET_ALT_M - alt
    p_deg   = math.degrees(pitch) if pitch is not None else 0.0
    elapsed = time.time() - track_start

    now = time.time()
    if now - last_print > 0.5:
        print(f"  {elapsed:6.1f}s  {alt:6.2f}m  {alt_err:+7.2f}m  "
              f"{TARGET_PITCH_DEG:8.1f}°  {p_deg:8.1f}°")
        last_print = now
    time.sleep(0.05)

print("\n  Attitude tracking complete.")

# ─────────────────────────────────────────────────────────────
# Step 7 – 降落前先將 PFA_DES_PITCH 歸零，避免著地時仍有傾斜指令
# ─────────────────────────────────────────────────────────────
print("\n[Step 7] Resetting PFA_DES_PITCH to 0° before landing...")
param_set('PFA_DES_PITCH', 0.0)
time.sleep(1.0)

print("Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
