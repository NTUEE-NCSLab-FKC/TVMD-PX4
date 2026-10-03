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

OFFBOARD signal lost 修正（offboard ↔ position 反覆切換）：
  PX4 只要 COM_OF_LOSS_T（預設 1 s）內沒收到 setpoint 就判定 offboard
  signal lost → failsafe 切 Position mode。舊版 setpoint 只在主迴圈發送，
  而主迴圈每圈都 recv_match(blocking=True) 等 LOCAL_POSITION_NED —— 14550
  為 GCS (normal) link，該訊息只有 1 Hz，setpoint 實際頻率 ≈ 1 Hz，剛好卡在
  1 s 超時邊緣；param_set() 等 ACK、wait_for_mode() 等 HEARTBEAT 時更是完全
  不發 setpoint。

  架構：
    _rx_worker  — 唯一讀 socket 的執行緒，更新遙測快取 / PARAM_VALUE / HEARTBEAT
    _sp_worker  — 唯一發 setpoint 的執行緒，固定 SP_RATE_HZ
    主執行緒    — 只讀快取、改 setpoint、送指令；任何等待都不會中斷 setpoint
  另外會印出模式切換（[MODE]）、PX4 STATUSTEXT（[PX4]）與 setpoint 間隔警告（[SP]），
  若 setpoint 無間斷仍掉 offboard，代表是 local position 失效（大傾角下估測器失效）
  而非通訊問題。

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
TARGET_ALT_M     = 1.0        # 懸停高度 (m)
TARGET_ROLL_DEG  = 45.0       # 期望 roll (deg，正值 = 右翼下)
TARGET_PITCH_DEG = 0.0        # pitch 維持水平

HOVER_PHASE_DUR  = 8.0
PITCH_RAMP_TIME  = 4.0        # roll ramp 時間 (s)
TRACK_PHASE_DUR  = 30.0

PARAM_STEP_DEG   = 5.0        # 每步進量 (deg，恆正)

ALT_TOL          = 0.15
HOVER_STABLE_TIME = 3.0

SP_RATE_HZ       = 20.0       # offboard setpoint 串流頻率 (Hz)，PX4 需 > 2 Hz

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


# 提高 LOCAL_POSITION_NED 串流頻率（GCS link 預設僅 1 Hz）
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    float(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED), 100_000.0,  # 10 Hz
    0, 0, 0, 0, 0)


# ─────────────────────────────────────────────────────────────
# Telemetry state cache（只由 _rx_worker 寫入，主執行緒只讀）
# ─────────────────────────────────────────────────────────────
_ned = {'x': 0.0, 'y': 0.0, 'z': 0.0, 't': 0.0}      # LOCAL_POSITION_NED
_att = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}       # ATTITUDE
_hb  = {'main_mode': None, 'sub_mode': None, 'armed': False, 't': 0.0}  # 飛控 HEARTBEAT
_params   = {}                         # PARAM_VALUE: name → (value, recv_time)
_param_cv = threading.Condition()

_PX4_MODE_NAMES = {1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: 'AUTO',
                   5: 'ACRO', 6: 'OFFBOARD', 7: 'STABILIZED', 8: 'RATTITUDE'}


def _handle_msg(msg):
    t = msg.get_type()
    if t == 'LOCAL_POSITION_NED':
        _ned['x'], _ned['y'], _ned['z'] = msg.x, msg.y, msg.z
        _ned['t'] = time.time()
    elif t == 'ATTITUDE':
        _att['roll'], _att['pitch'], _att['yaw'] = msg.roll, msg.pitch, msg.yaw
    elif t == 'HEARTBEAT':
        # 只看飛控本身的 heartbeat（排除 GCS / MAVProxy / companion）
        if (msg.get_srcSystem() != master.target_system
                or msg.type == mavutil.mavlink.MAV_TYPE_GCS
                or msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID):
            return
        main_mode = (msg.custom_mode >> 16) & 0xFF
        sub_mode  = (msg.custom_mode >> 24) & 0xFF
        if _hb['main_mode'] is not None and main_mode != _hb['main_mode']:
            print(f"\n  [MODE] {_PX4_MODE_NAMES.get(_hb['main_mode'], _hb['main_mode'])}"
                  f" → {_PX4_MODE_NAMES.get(main_mode, main_mode)}"
                  f"  (setpoint 最大間隔 {_sp_stat['max_gap']*1000:.0f} ms)", flush=True)
        _hb['main_mode'], _hb['sub_mode'] = main_mode, sub_mode
        _hb['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        _hb['t'] = time.time()
    elif t == 'PARAM_VALUE':
        with _param_cv:
            _params[msg.param_id.rstrip('\x00')] = (msg.param_value, time.time())
            _param_cv.notify_all()
    elif t == 'STATUSTEXT':
        print(f"\n  [PX4] {msg.text.rstrip(chr(0))}", flush=True)


def _rx_worker():
    """唯一讀取 socket 的執行緒。主執行緒完全不呼叫 recv_match，
    因此任何等待（ACK、模式、arm）都不會卡住 setpoint 串流。"""
    while _running.is_set():
        try:
            msg = master.recv_match(blocking=True, timeout=0.1)
        except Exception as e:          # 單一壞封包不應終止接收執行緒
            print(f"\n  [RX] error: {e}", flush=True)
            continue
        if msg is not None and msg.get_type() != 'BAD_DATA':
            _handle_msg(msg)


# ─────────────────────────────────────────────────────────────
# Background setpoint thread
# 唯一負責 offboard setpoint 的執行緒，固定 SP_RATE_HZ 發送，與主迴圈完全解耦。
# ─────────────────────────────────────────────────────────────
_tx_lock   = threading.Lock()      # pymavlink 的 mav.send() 非 thread-safe（seq 計數）
_running   = threading.Event()     # rx / setpoint 執行緒存活旗標
_running.set()
_sp_z      = -TARGET_ALT_M         # 目前 setpoint z（主執行緒經 set_position_target 更新）
_sp_active = threading.Event()
_sp_active.set()
_sp_stat   = {'last': 0.0, 'max_gap': 0.0}


def _send(fn, *args):
    with _tx_lock:
        fn(*args)


def _sp_worker():
    period = 1.0 / SP_RATE_HZ
    next_t = time.monotonic()
    while _running.is_set():
        if _sp_active.is_set():
            # position + yaw (bit10=0)，ignore vel/acc/yaw_rate；yaw=0.0 (有限值，防 NaN)
            _send(master.mav.set_position_target_local_ned_send,
                  0, master.target_system, master.target_component,
                  mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                  0b0000101111111000,
                  0.0, 0.0, _sp_z,
                  0, 0, 0,
                  0, 0, 0,
                  0.0, 0)
            now = time.monotonic()
            if _sp_stat['last']:
                gap = now - _sp_stat['last']
                _sp_stat['max_gap'] = max(_sp_stat['max_gap'], gap)
                if gap > 0.3:
                    print(f"\n  [SP] WARNING: setpoint gap {gap*1000:.0f} ms", flush=True)
            _sp_stat['last'] = now
        # 以絕對時間排程，避免 sleep 誤差累積；落後太多則重新對齊
        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()


_rx_thread = threading.Thread(target=_rx_worker, daemon=True)
_sp_thread = threading.Thread(target=_sp_worker, daemon=True)
_rx_thread.start()
_sp_thread.start()


# ─────────────────────────────────────────────────────────────
# Helpers（皆不讀 socket，只讀 _rx_worker 的快取）
# ─────────────────────────────────────────────────────────────

def param_set(name, value, retries=5, timeout=1.5):
    """PARAM_SET + 等待本次送出後的 PARAM_VALUE ACK，並確認值一致。"""
    nb = name.encode('utf-8')
    for _ in range(retries):
        t_sent = time.time()
        _send(master.mav.param_set_send,
              master.target_system, master.target_component,
              nb, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        with _param_cv:
            got = _param_cv.wait_for(
                lambda: name in _params and _params[name][1] >= t_sent, timeout)
            if got and abs(_params[name][0] - value) < 0.01:
                return True
    return False


def param_get(name, timeout=3.0):
    t_sent = time.time()
    _send(master.mav.param_request_read_send,
          master.target_system, master.target_component,
          name.encode('utf-8'), -1)
    with _param_cv:
        if _param_cv.wait_for(
                lambda: name in _params and _params[name][1] >= t_sent, timeout):
            return _params[name][0]
    return None


def set_position_target(x, y, z):
    """更新 setpoint z；實際發送由 _sp_worker 負責（x, y 固定 0）。"""
    global _sp_z
    _sp_z = z


def get_local_position():
    """回傳最新 LOCAL_POSITION_NED 的 (x, y, z)；尚未收到則為 None。"""
    if _ned['t'] == 0.0:
        return None, None, None
    return _ned['x'], _ned['y'], _ned['z']


def get_attitude():
    """Returns (roll, pitch, yaw) in radians."""
    return _att['roll'], _att['pitch'], _att['yaw']


def wait_until(cond, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def wait_for_mode(target_main_mode, timeout=5):
    return wait_until(lambda: _hb['main_mode'] == target_main_mode, timeout)


def wait_for_armed(timeout=5):
    return wait_until(lambda: _hb['armed'], timeout)


def set_mode_offboard():
    _send(master.mav.command_long_send,
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
          6.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def set_mode_land():
    _send(master.mav.command_long_send,
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
          4.0, 6.0, 0.0, 0.0, 0.0, 0.0)


def arm(force=False):
    p2 = 21196.0 if force else 0.0
    _send(master.mav.command_long_send,
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
wait_until(lambda: _ned['t'] > 0.0, 5.0)
x, y, z = get_local_position()
if x is None:
    print("ERROR: No LOCAL_POSITION_NED.")
    _running.clear()
    sys.exit(1)
print(f"  OK – NED position: ({x:.2f}, {y:.2f}, {z:.2f})")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Streaming initial setpoints (5 s)...")
set_position_target(0.0, 0.0, -TARGET_ALT_M)   # _sp_worker 已在背景持續發送
time.sleep(5.0)

print("  Requesting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD confirmed.")
else:
    time.sleep(2.5)
    set_mode_offboard()
    print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: not confirmed.")

print("  Arming...")
time.sleep(1.0)
arm()

armed = wait_for_armed(5.0)
if not armed:
    arm(force=True)
    armed = wait_for_armed(5.0)

if not armed:
    print("ERROR: Failed to arm.")
    _running.clear()
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
    _, _, pz = get_local_position()
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
    now = time.time()
    if now - last_print > 2.0:
        _, _, pz = get_local_position()
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
        _, _, pz = get_local_position()
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
    _, _, pz = get_local_position()
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
# 確認已離開 OFFBOARD 才停止 setpoint，否則會先觸發 offboard lost → Position mode
if not wait_for_mode(4, 5.0):
    print("  WARNING: AUTO.LAND not confirmed, retrying...")
    set_mode_land()
    wait_for_mode(4, 5.0)
_sp_active.clear()   # 停止 setpoint 串流
time.sleep(15)
_running.clear()
print("Done.")
