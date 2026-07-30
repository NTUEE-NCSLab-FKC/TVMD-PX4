#!/usr/bin/env python3
"""
PFA Pitch Tracking Task (pymavlink) — non-blocking 版本
=========================================================
懸停於 TARGET_ALT_M，pitch 漸進至 TARGET_PITCH_DEG 並維持追蹤。

修正：
  1. drain() — 非阻塞排空訊息緩衝，讓飛行迴圈不因 recv_match(blocking=True)
               中斷 setpoint 流，避免 OFFBOARD 500ms 超時掉出。
  2. 背景 setpoint 執行緒 — param_set() 本身需等 PARAM_VALUE ACK (~1.5 s)，
               期間主執行緒無法發送 setpoint。背景執行緒確保 setpoint 在
               param_set 阻塞期間仍以 25 Hz 持續發出。

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
import threading
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 0.4     # 懸停高度 (m)
TARGET_PITCH_DEG  = 30.0    # 期望 pitch (deg, 正值 = nose up)
TARGET_ROLL_DEG   = 0.0

HOVER_PHASE_DUR   = 8.0     # 穩定後的水平懸停保持時間 (s)
PITCH_RAMP_TIME   = 4.0     # pitch 從 0° 爬升到目標角度的總時間 (s)
TRACK_PHASE_DUR   = 30.0    # 維持目標 pitch 的追蹤時長 (s)
PARAM_STEP_DEG    = 5.0     # 每次 PARAM_SET 的步進量 (deg)

ALT_TOL           = 0.15    # 懸停高度容差 (m)
HOVER_STABLE_TIME = 3.0     # 判定穩定懸停所需的持續時間 (s)

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

# 請求 PX4 串流 SERVO_OUTPUT_RAW（msg 36）和 ACTUATOR_OUTPUT_STATUS（msg 375）
# PX4 預設不主動發送這兩種訊息，必須用 MAV_CMD_SET_MESSAGE_INTERVAL 訂閱
_STREAM_RATE_US = 100_000   # 10 Hz = 100 ms
for _msg_id in (36, 375):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(_msg_id), float(_STREAM_RATE_US),
        0, 0, 0, 0, 0)
    time.sleep(0.05)
print("  Requested SERVO_OUTPUT_RAW + ACTUATOR_OUTPUT_STATUS streams (10 Hz)")


# ─────────────────────────────────────────────────────────────
# Telemetry state cache  (由 drain() 更新)
# ─────────────────────────────────────────────────────────────
_ned    = {'x': 0.0, 'y': 0.0, 'z': -0.01}       # LOCAL_POSITION_NED
_att    = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}  # ATTITUDE (actual)
_att_sp = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}  # ATTITUDE_TARGET (pfa_pos_control setpoint)
# SERVO_OUTPUT_RAW: port=0 → 4 motors (Main PWM), port=1 → 8 servos (Aux PWM)
# SITL: 全部輸出在 port=0（ch0-3=motors, ch4-11=servos）
# 實機: motors=port=0, servos=port=1(AUX)
# Motor throttle: (pwm - 1000) / 10.0 [%];  Servo tilt: ±30°, center=1500µs, range=±300µs
_act = {'motors': [], 'servos': [], 'sitl_detected': False}  # SERVO_OUTPUT_RAW


def _quat_to_euler(q):
    """四元數 [w, x, y, z] → (roll, pitch, yaw) rad。"""
    w, x, y, z = q[0], q[1], q[2], q[3]
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def drain():
    """非阻塞排空 socket 緩衝，更新 _ned / _att / _att_sp / _act。
    在飛行迴圈最前面呼叫，確保 recv_match 不占用 setpoint 發送視窗。
    """
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == 'LOCAL_POSITION_NED':
            _ned['x'], _ned['y'], _ned['z'] = msg.x, msg.y, msg.z
        elif t == 'ATTITUDE':
            _att['roll'], _att['pitch'], _att['yaw'] = msg.roll, msg.pitch, msg.yaw
        elif t == 'ATTITUDE_TARGET':
            r, p, y = _quat_to_euler(msg.q)
            _att_sp['roll'], _att_sp['pitch'], _att_sp['yaw'] = r, p, y
        elif t == 'SERVO_OUTPUT_RAW':
            ch = [msg.servo1_raw,  msg.servo2_raw,  msg.servo3_raw,  msg.servo4_raw,
                  msg.servo5_raw,  msg.servo6_raw,  msg.servo7_raw,  msg.servo8_raw,
                  msg.servo9_raw,  msg.servo10_raw, msg.servo11_raw, msg.servo12_raw]
            if msg.port == 0:
                # MAIN PWM: Motor 0-3
                # SITL では port=0 にすべて含まれる場合あり（Motor + Servo 合計 12ch）
                _act['motors'] = ch[:4]
                # Servo は ch[4..11] だが、実機では idle=1000µs で舵機ではない
                # → 実機(port=1 あり)は port=1 から取得するため、ここでは上書きしない
                if not _act['sitl_detected'] and any(
                        1100 < v < 1900 for v in ch[4:12]):
                    _act['sitl_detected'] = True
                if _act['sitl_detected']:
                    _act['servos'] = ch[4:12]
            elif msg.port == 1:
                # AUX PWM (hardware): Servo 0-7
                _act['sitl_detected'] = False   # port=1 あり → 実機モード確定
                _act['servos'] = ch[:8]
        elif t == 'ACTUATOR_OUTPUT_STATUS':
            # 備用：ACTUATOR_OUTPUT_STATUS 也攜帶全部 actuator 輸出（normalized）
            # 若 SERVO_OUTPUT_RAW 未收到，用此作為替代
            n = bin(msg.active).count('1') if msg.active else 0
            if n >= 12 and not _act['motors']:
                # normalized 0..1 for motors → 偽 PWM µs
                _act['motors'] = [int(msg.actuator[i] * 1000 + 1000) for i in range(4)]
                _act['servos'] = [int(msg.actuator[i] * 300  + 1500) for i in range(4, 12)]


def _fmt_act() -> str:
    """格式化馬達油門 % 和舵機角度，用於 log 輸出。
    舵機範圍 ±30°，1200µs=-30°，1500µs=0°，1800µs=+30°。
    每 Module 兩顆：servo[2k]=X軸，servo[2k+1]=Y軸。
    """
    m, s = _act['motors'], _act['servos']
    if not m or not s:
        return "(no actuator data)"
    pct = [(v - 1000) / 10.0 for v in m]
    motor_s = f"motors {pct[0]:.0f}%,{pct[1]:.0f}%,{pct[2]:.0f}%,{pct[3]:.0f}%"
    ang = [(v - 1500) / 300.0 * 30.0 for v in s]
    servo_s = (f"servos(X°,Y°) "
               f"M0({ang[0]:+.1f},{ang[1]:+.1f}) "
               f"M1({ang[2]:+.1f},{ang[3]:+.1f}) "
               f"M2({ang[4]:+.1f},{ang[5]:+.1f}) "
               f"M3({ang[6]:+.1f},{ang[7]:+.1f})")
    return f"{motor_s}  {servo_s}"


# ─────────────────────────────────────────────────────────────
# Background setpoint thread
# 在 param_set() 阻塞期間維持 25 Hz setpoint，防止 OFFBOARD 超時掉出
# ─────────────────────────────────────────────────────────────
_sp_z      = -TARGET_ALT_M     # 目前 setpoint z（主執行緒可更新）
_sp_active = threading.Event()
_sp_active.set()


def _sp_worker():
    while _sp_active.is_set():
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            0.0, 0.0, _sp_z,
            0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.04)    # 25 Hz


_sp_thread = threading.Thread(target=_sp_worker, daemon=True)
_sp_thread.start()


# ─────────────────────────────────────────────────────────────
# MAVLink helpers
# ─────────────────────────────────────────────────────────────

def set_position_target(z: float = None):
    """發送 NED 位置 setpoint，z 預設用 _sp_z。主執行緒呼叫版本。"""
    global _sp_z
    if z is not None:
        _sp_z = z
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        0.0, 0.0, _sp_z,
        0, 0, 0, 0, 0, 0, 0, 0)


def param_set(name: str, value: float, retries: int = 5) -> bool:
    """PARAM_SET + 等待 PARAM_VALUE ACK，並驗證 FCU 回傳值與設定值一致。
    背景執行緒在此期間持續發送 setpoint，防止 OFFBOARD 掉出。

    成功條件：
      1. param_id 符合（排除 MAVProxy 轉發的其他參數 ACK）
      2. param_value 與 value 誤差 < 0.01（排除舊快取造成的假 OK）
    """
    nb = name.encode('utf-8')
    for attempt in range(retries):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            nb, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if ack and ack.param_id.rstrip('\x00') == name:
            actual = ack.param_value
            if abs(actual - value) < 0.01:
                print(f"    FCU ACK: {name} = {actual:.4f}  (sent {value:.4f})  ✓")
                return True
            else:
                print(f"    FCU ACK mismatch: {name} = {actual:.4f}  (sent {value:.4f})"
                      f"  retry {attempt+1}/{retries}")
        time.sleep(0.3)
    return False


def param_get(name: str):
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), -1)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg and msg.param_id.rstrip('\x00') == name:
        return msg.param_value
    return None


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
        6.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def set_mode_land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0)


def arm(force: bool = False):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0 if force else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


# ─────────────────────────────────────────────────────────────
# Step 0 – 設定 PFA_DES_PITCH / PFA_DES_ROLL 參數
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Configuring PFA attitude target parameters...")

current_pitch_param = param_get('PFA_DES_PITCH')
if current_pitch_param is None:
    print("  WARNING: Could not read PFA_DES_PITCH. Check firmware.")
else:
    print(f"  PFA_DES_PITCH current = {current_pitch_param:.1f}°")

print("  Setting PFA_DES_ROLL  = 0°  ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_ROLL', 0.0) else "FAILED (continuing)")

print("  Setting PFA_DES_PITCH = 0°  ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED (continuing)")

# ─────────────────────────────────────────────────────────────
# Step 1 – Pre-flight check
# ─────────────────────────────────────────────────────────────
print("\n[Step 1] Checking local position estimate...")
msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
if msg is None:
    print("  ERROR: No LOCAL_POSITION_NED received. Is EKF2 running?")
    _sp_active.clear()
    sys.exit(1)
_ned['x'], _ned['y'], _ned['z'] = msg.x, msg.y, msg.z
print(f"  OK – NED position: ({_ned['x']:.2f}, {_ned['y']:.2f}, {_ned['z']:.2f})")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Streaming initial setpoints (5 s, z={-TARGET_ALT_M:.1f} NED)...")
set_position_target(-TARGET_ALT_M)   # 更新 _sp_z，背景執行緒開始發
time.sleep(5.0)

print("  Requesting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD confirmed.")
else:
    time.sleep(2.5)
    set_mode_offboard()
    print("  OFFBOARD confirmed." if wait_for_mode(6)
          else "  WARNING: not confirmed, continuing.")

print("  Arming...")
time.sleep(1.0)
arm()

armed = False
for _ in range(100):
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    arm(force=True)
    for _ in range(100):
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.05)

if not armed:
    print("  ERROR: Failed to arm.")
    _sp_active.clear()
    sys.exit(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – 穩定懸停至 TARGET_ALT_M（PFA_DES_PITCH = 0°）
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Position-control hover at {TARGET_ALT_M} m  (PFA_DES_PITCH=0°)")
print(f"  Waiting for stable hover (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

phase_start  = time.time()
stable_since = None
last_print   = 0.0

while True:
    drain()                              # ← 非阻塞更新快取
    set_position_target()               # ← 主執行緒補發（背景已在發）

    alt     = -_ned['z']
    alt_err = TARGET_ALT_M - alt
    p_deg   = math.degrees(_att['pitch'])
    stable_since = (stable_since or time.time()) if abs(alt_err) < ALT_TOL else None

    now = time.time()
    if now - last_print > 1.0:
        s = now - stable_since if stable_since else 0.0
        print(f"  alt={alt:.2f} m (err={alt_err:+.2f})  pitch={p_deg:.1f}°  "
              f"stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
        last_print = now

    if stable_since and (time.time() - stable_since) >= HOVER_STABLE_TIME:
        print(f"  Stable hover at {alt:.2f} m.")
        break
    if time.time() - phase_start > 25.0:
        print(f"  Hover timeout – alt={alt:.2f} m, continuing.")
        break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 4 – 保持水平懸停 HOVER_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
end_t      = time.time() + HOVER_PHASE_DUR
last_print = 0.0

while time.time() < end_t:
    drain()
    set_position_target()
    now = time.time()
    if now - last_print > 2.0:
        alt   = -_ned['z']
        p_deg = math.degrees(_att['pitch'])
        print(f"  alt={alt:.2f} m  pitch={p_deg:.1f}°  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 5 – 漸進式 pitch ramp（逐步 PARAM_SET PFA_DES_PITCH）
#
# 背景執行緒持續發 setpoint，param_set 的 ACK 等待（~1.5 s）
# 不再造成 setpoint 中斷，OFFBOARD 不會因此掉出。
# ─────────────────────────────────────────────────────────────
PARAM_STEP_WAIT = PITCH_RAMP_TIME / (abs(TARGET_PITCH_DEG) / PARAM_STEP_DEG)
_sat_limit = math.degrees(math.asin(0.3 / ((1.4 * 9.81) / 24.0)))

print(f"\n[Step 5] Pitch ramp: 0° → {TARGET_PITCH_DEG:.0f}° "
      f"(step={PARAM_STEP_DEG:.0f}°, interval={PARAM_STEP_WAIT:.1f} s/step)")
print(f"  XY thrust saturation limit: ±{_sat_limit:.1f}°  "
      f"({'OK' if abs(TARGET_PITCH_DEG) <= _sat_limit else 'WARNING: near/over saturation'})")

current_pitch_cmd = 0.0

while abs(current_pitch_cmd - TARGET_PITCH_DEG) > 0.01:
    step       = math.copysign(PARAM_STEP_DEG, TARGET_PITCH_DEG - current_pitch_cmd)
    next_pitch = current_pitch_cmd + step
    if (step > 0 and next_pitch > TARGET_PITCH_DEG) or \
       (step < 0 and next_pitch < TARGET_PITCH_DEG):
        next_pitch = TARGET_PITCH_DEG

    print(f"  PARAM_SET PFA_DES_PITCH = {next_pitch:.1f}° ... ", end='', flush=True)
    ok = param_set('PFA_DES_PITCH', next_pitch)   # 背景執行緒在此期間維持 setpoint
    if ok:
        # 等待 pfa_pos_control 一個 Run() 循環 + 馬達響應
        time.sleep(0.15)
        drain()
        sp_deg = math.degrees(_att_sp['pitch'])
        # vehicle_attitude_setpoint (via ATTITUDE_TARGET MAVLink stream)
        # 在 TVMD 6DOF 架構下此值可能維持 0°（控制路徑走 thrust/torque setpoint）
        print(f"  vehicle_attitude_setpoint pitch = {sp_deg:.1f}°  (from ATTITUDE_TARGET)")
        print(f"  {_fmt_act()}")
    else:
        print("  PARAM_SET FAILED")
    current_pitch_cmd = next_pitch

    # 等待姿態響應，同時持續發 setpoint
    step_end   = time.time() + PARAM_STEP_WAIT
    last_print = 0.0
    while time.time() < step_end:
        drain()
        set_position_target()

        alt    = -_ned['z']
        p_deg  = math.degrees(_att['pitch'])
        sp_deg = math.degrees(_att_sp['pitch'])
        err    = p_deg - current_pitch_cmd
        now    = time.time()
        if now - last_print > 0.5:
            print(f"    alt={alt:.2f} m  pitch_cmd={current_pitch_cmd:.1f}°  "
                  f"veh_att_sp={sp_deg:.1f}°  pitch_now={p_deg:.1f}°  err={err:+.1f}°")
            last_print = now
        time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 6 – 維持 pitch=TARGET_PITCH_DEG，持續追蹤 TRACK_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 6] Holding pitch={TARGET_PITCH_DEG:.0f}°, alt={TARGET_ALT_M} m "
      f"for {TRACK_PHASE_DUR:.0f} s...")
print(f"  {'Time':>6}  {'Alt':>6}  {'AltErr':>7}  {'PitchCmd':>9}  {'VehAttSp':>9}  {'PitchNow':>9}  {'PitchErr':>9}")
print("  " + "─" * 72)

track_start    = time.time()
last_print     = 0.0
last_act_print = 0.0   # 每 5 s 輸出一次完整 actuator 狀態

while time.time() - track_start < TRACK_PHASE_DUR:
    drain()
    set_position_target()

    alt     = -_ned['z']
    alt_err = TARGET_ALT_M - alt
    p_deg   = math.degrees(_att['pitch'])
    sp_deg  = math.degrees(_att_sp['pitch'])
    p_err   = p_deg - TARGET_PITCH_DEG
    elapsed = time.time() - track_start

    now = time.time()
    if now - last_print > 0.5:
        print(f"  {elapsed:6.1f}s  {alt:6.2f}m  {alt_err:+7.2f}m  "
              f"{TARGET_PITCH_DEG:8.1f}°  {sp_deg:8.1f}°  {p_deg:8.1f}°  {p_err:+8.1f}°")
        last_print = now

    if now - last_act_print > 5.0:
        print(f"  [Act @ {elapsed:.0f}s] {_fmt_act()}")
        last_act_print = now

    time.sleep(0.05)

print("\n  Attitude tracking complete.")

# ─────────────────────────────────────────────────────────────
# Step 7 – 降落前將 PFA_DES_PITCH 歸零，切換 AUTO.LAND
# ─────────────────────────────────────────────────────────────
print("\n[Step 7] Resetting PFA_DES_PITCH to 0° before landing...")
param_set('PFA_DES_PITCH', 0.0)
time.sleep(1.0)

_sp_active.clear()   # 停止背景 setpoint 執行緒

print("Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
