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

OFFBOARD signal lost 修正（offboard ↔ position 反覆切換）：
  PX4 只要 COM_OF_LOSS_T（預設 1 s）內沒收到 setpoint 就判定 offboard
  signal lost → failsafe 切 Position mode。舊版 setpoint 只在主迴圈發送，
  而主迴圈每圈都 recv_match(blocking=True) 等 LOCAL_POSITION_NED —— 14550
  為 GCS (normal) link，該訊息只有 1 Hz（經數傳電台降速後更只剩 ~0.4 Hz），
  setpoint 實際頻率 ≈ 1 Hz 以下；param_set()/wait_for_mode() 等待時完全不發。

  架構（與 pfa_roll_tracking.py / pfa_pitch_tracking.py 相同）：
    _rx_worker  — 唯一讀 socket 的執行緒，更新遙測快取 / PARAM_VALUE / HEARTBEAT
    _sp_worker  — 唯一發 setpoint 的執行緒，固定 SP_RATE_HZ（含目前 yaw 指令）
    主執行緒    — 只讀快取、改 setpoint（z / yaw）、送指令

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
TARGET_ALT_M     = 1.0     # 懸停高度 (m)
TARGET_YAW_DEG   = 90.0    # 目標 yaw (deg, NED: +90=East, 順時針)
TARGET_ROLL_DEG  = 0.0     # roll/pitch 維持 0°
TARGET_PITCH_DEG = 0.0

HOVER_PHASE_DUR  = 8.0     # 水平懸停穩定後等待 (s)
YAW_RAMP_TIME    = 4.0     # yaw ramp 持續時間 (s)
TRACK_PHASE_DUR  = 30.0    # 維持目標 yaw 的追蹤時長 (s)

ALT_TOL          = 0.15
HOVER_STABLE_TIME = 3.0

# offboard setpoint 串流頻率 (Hz)。PX4 只要求間隔 < COM_OF_LOSS_T (1 s)；
# 經 57600 baud 數傳 (MAV_0_RATE=1200 B/s) 時每筆 65 B，10 Hz ≈ 650 B/s 已足夠且不壅塞鏈路
SP_RATE_HZ       = 10.0

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


# 提高 LOCAL_POSITION_NED 串流頻率（GCS link 預設僅 1 Hz；數傳頻寬有限，取 5 Hz）
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    float(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED), 200_000.0,  # 5 Hz
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
_sp_yaw    = 0.0                   # 目前 setpoint yaw (rad, NED)
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
            # position + yaw (bit10=0)，ignore vel/acc/yaw_rate；yaw 恆為有限值（防 NaN）
            _send(master.mav.set_position_target_local_ned_send,
                  0, master.target_system, master.target_component,
                  mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                  0b0000101111111000,
                  0.0, 0.0, _sp_z,
                  0, 0, 0,
                  0, 0, 0,
                  float(_sp_yaw), 0)
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


def set_position_target(x, y, z, yaw_rad=0.0):
    """更新 setpoint z / yaw；實際發送由 _sp_worker 負責（x, y 固定 0）。"""
    global _sp_z, _sp_yaw
    _sp_z, _sp_yaw = z, yaw_rad


def get_local_position():
    """回傳最新 LOCAL_POSITION_NED 的 (x, y, z)；尚未收到則為 None。"""
    if _ned['t'] == 0.0:
        return None, None, None
    return _ned['x'], _ned['y'], _ned['z']


def get_attitude():
    """Returns (roll, pitch, yaw) in radians (NED convention)."""
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
INIT_YAW_RAD = 0.0   # 起飛時送 yaw=0°（朝北），確保有限值

print(f"\n[Step 2] Streaming initial setpoints (5 s, yaw=0°)...")
set_position_target(0.0, 0.0, -TARGET_ALT_M, yaw_rad=INIT_YAW_RAD)  # _sp_worker 已在背景持續發送
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
# Step 3 – 位置控制懸停至 1 m (yaw=0°)
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Hover at {TARGET_ALT_M} m  (yaw=0°, PFA_DES_ROLL/PITCH=0°)")
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
    now = time.time()
    if now - last_print > 2.0:
        _, _, pz = get_local_position()
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
        _, _, pz = get_local_position()
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
    _, _, pz = get_local_position()
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
# 確認已離開 OFFBOARD 才停止 setpoint，否則會先觸發 offboard lost → Position mode
if not wait_for_mode(4, 5.0):
    print("  WARNING: AUTO.LAND not confirmed, retrying...")
    set_mode_land()
    wait_for_mode(4, 5.0)
_sp_active.clear()   # 停止 setpoint 串流
time.sleep(15)
_running.clear()
print("Done.")
