#!/usr/bin/env python3
"""
PFA Path Tracking — DEBUG_VECT stream (pymavlink)
==================================================

pfa_path_tracking.py 的串流版：直線前進，先 roll ±ROLL_AMPLITUDE_DEG × NUM_ROLL_CYCLES，
水平 PAUSE_S 秒，再 pitch ±PITCH_AMPLITUDE_DEG × NUM_PITCH_CYCLES，前進不中斷。
  θ(t) = A·sin(2π (t − t_seg) / OSCILLATION_PERIOD_S)，每段從 0° 開始、整數週期後回到 0°
  時間軸：LEAD_IN_S 加速 → roll 段 → PAUSE_S 水平 → pitch 段 → LEAD_OUT_S 減速

■ 為什麼改寫（舊版會飛回原點）
  舊版的 param_set() 等 PARAM_VALUE 時完全不送 setpoint；擺動期間每秒 10 次
  PARAM_SET 會觸發參數存檔，確認常延遲到 0.5–1 s。SITL 的 COM_OF_LOSS_T = 0.5 s
  → offboard lost → COM_OBL_RC_ACT = 0 要切 Position mode，但沒有遙控器 →
  NAV_RCL_ACT = 2 → RTL 飛回 home（原點）。本版：
  - _sp_worker 執行緒固定 SP_RATE_HZ 送 setpoint，主執行緒任何等待都不會中斷串流
  - _rx_worker 執行緒是唯一讀 socket 的地方（不再有阻塞的 recv_match）
  - 飛行中不送任何 PARAM_SET：姿態以 DEBUG_VECT "att_cmd" 串流（PFA_ATT_SRC = 1）

■ 姿態指令（DEBUG_VECT 串流）
  每拍送 DEBUG_VECT(name="att_cmd", x = roll°, y = pitch°, z = 取樣時間 t [s])。
  pfa_pos_control 以最近兩筆的 Δθ/Δz 算角速度，封包之間外插（PFA_ATT_EXTR），
  超過 PFA_ATT_TOUT 沒收到就以 PFA_ATT_SLEW 漸變回 PFA_DES_ROLL/PITCH（= 0）。
  姿態讓步（PFA_YIELD_TILT）仍然有效：超出可行範圍的角度會被限制，位置與高度優先。
  CMD_LEAD_S > 0 時送 t + CMD_LEAD_S 的指令，補償姿態迴路約 0.4 s 的延遲。

■ 姿態偏差補償（BIAS_*）
  指令 0° 懸停時實測姿態有穩態偏差 b（SITL 14_38_27：roll ≈ +1.0°、pitch ≈ +2.4°；
  重心 / 力矩干擾，姿態迴路沒有積分）。開始前進前先以 0° 懸停 BIAS_MEASURE_S 秒量測
  平均偏差 b，之後串流送出
    送出值(t) = 參考軌跡(t + CMD_LEAD_S) − b
  讓「實測」姿態以真正的 0° 為中心擺動。量完後先送 −b 懸停 BIAS_SETTLE_S 秒，
  印出補償後的殘差再開始前進；終點懸停期間也維持 −b，降落前才關串流。
  補償只移動指令，干擾力矩仍在，兩側可用的舵機角度不對稱。統計的誤差是 參考 − 實測。

■ 前進軌跡（NED，以起飛點正上方為起點）
  沿 PATH_YAW_DEG 方向直線前進，速度 v(t) 在前後 SPEED_RAMP_S 以 smoothstep 加減速，
  中段等速 PATH_SPEED_MPS；送 位置 + 速度 + 加速度 前饋，yaw 固定 PATH_YAW_DEG。
  懸停時若起飛航向與 PATH_YAW_DEG 不同，先以最小 jerk 曲線轉過去。

■ 數傳頻寬（57600 baud，每方向約 5.7 kB/s）
  上行：DEBUG_VECT 約 42 B × CMD_HZ（30 Hz ≈ 1.3 kB/s）+ setpoint 約 65 B × SP_RATE_HZ
  （20 Hz ≈ 1.3 kB/s）。SITL（UDP）可把 CMD_HZ 提高到 50。

■ 安全
  - 期間若離開 OFFBOARD 或被上鎖：停止串流、PFA_ATT_SRC 設回 0，不再切換模式
  - Ctrl-C：姿態串流歸零、PFA_ATT_SRC 設回 0，原地懸停後 AUTO.LAND
  - 腳本當掉或斷線：韌體在 PFA_ATT_TOUT 後自己漸變回參數值（0°）
  - OFFBOARD 下必須 PFA_TKF_BYP = 1（啟動時會檢查）

Usage: python3 pfa_path_tracking_stream.py [connection]   (預設 udp:127.0.0.1:14550；SITL 可用 udpin:0.0.0.0:14540)
"""

import sys
import time
import math
import struct
import threading
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
CONNECTION        = sys.argv[1] if len(sys.argv) > 1 else 'udp:127.0.0.1:14550'

TARGET_ALT_M      = 1.0    # 飛行高度 (m)
PATH_SPEED_MPS    = 0.5    # 前進速度 (m/s)
PATH_YAW_DEG      = 0.0    # 前進方向與機頭方向 (deg，NED：0 = 北、90 = 東)；None = 起飛時的航向
SPEED_RAMP_S      = 2.0    # 前進速度的加速 / 減速時間 (s)

ROLL_AMPLITUDE_DEG   = 30.0   # roll 振盪幅度 (deg)，正值 = 右翼朝下
PITCH_AMPLITUDE_DEG  = 30.0   # pitch 振盪幅度 (deg)，正值 = 機頭朝上
OSCILLATION_PERIOD_S = 6.0    # 振盪週期 (s)；峰值角速度 = 2π·A/T（30°, 6 s → 31°/s）
NUM_ROLL_CYCLES      = 2      # roll 完整振盪次數
NUM_PITCH_CYCLES     = 2      # pitch 完整振盪次數
PAUSE_S              = 2.0    # roll 與 pitch 之間的水平段 (s)，期間繼續前進
LEAD_IN_S            = SPEED_RAMP_S   # 開始前進到開始擺動 (s)：等速度到達 PATH_SPEED_MPS
LEAD_OUT_S           = SPEED_RAMP_S   # 擺動結束到停止 (s)
MAX_AMP_DEG       = 60.0   # 振幅硬上限 (deg)

CMD_HZ            = 30.0   # DEBUG_VECT "att_cmd" 送出頻率 (Hz)；SITL 可 50，57600 數傳建議 ≤ 30
CMD_LEAD_S        = 0.4    # 指令提前量 (s)：補償姿態迴路約 0.4 s 的延遲（SITL 實測 0.37–0.42 s）；0 = 不補償
# 韌體串流參數；None = 不改 FCU 目前的值
ATT_TOUT_S        = None   # PFA_ATT_TOUT (s，預設 0.5)
ATT_EXTR_S        = None   # PFA_ATT_EXTR (s，預設 0.3)
ATT_SLEW_DPS      = None   # PFA_ATT_SLEW (deg/s，預設 60)

HOVER_BEFORE_S    = 5.0    # 開始前的水平懸停時間 (s)

# 姿態偏差補償：送出值 = 參考 − 偏差 b（b = 指令 0° 懸停時的實測平均姿態）
BIAS_COMP         = True   # False = 不補償
BIAS_MEASURE_S    = 5.0    # 量測時間 (s)：PFA_DES_ROLL/PITCH = 0°、無串流時懸停，取實測平均
BIAS_MAX_DEG      = 8.0    # 偏差上限 (deg)：超過視為異常，補償量限制在 ±BIAS_MAX_DEG
BIAS_SETTLE_S     = 3.0    # 送 −b 後等待姿態穩定、再驗證殘差的時間 (s)
HOLD_END_S        = 5.0    # 結束後在終點的懸停時間 (s)
ALT_TOL           = 0.15   # 高度穩定容許誤差 (m)
HOVER_STABLE_TIME = 3.0    # 高度在容許範圍內需持續的時間 (s)
ALT_WAIT_TIMEOUT  = 25.0   # 等待高度穩定的上限 (s)

# 平滑起飛
CLIMB_PEAK_MPS    = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S       = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S  = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)
YAW_PEAK_RATE_DPS = 45.0   # 懸停時轉向的峰值角速度 (deg/s)
YAW_MIN_TURN_S    = 1.0    # 轉向最短時間 (s)
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × 距離 / 時間

SP_RATE_HZ        = 20.0   # offboard setpoint 串流頻率 (Hz)
LOG_PERIOD_S      = 0.5    # 追蹤狀態印出間隔 (s)
PX4_SPEED_XY_MAX  = 1.0    # pfa_pos_control 的 _speed_xy_max (m/s)

# 遙測串流 (msg id → Hz)：ATTITUDE、ATTITUDE_TARGET（讓步後的 setpoint）10 Hz、
# LOCAL_POSITION_NED 5 Hz、SERVO_OUTPUT_RAW 2 Hz
_STREAMS_HZ = {30: 10.0, 83: 10.0, 32: 5.0, 36: 2.0}

_OMEGA          = 2.0 * math.pi / OSCILLATION_PERIOD_S
_T_ROLL0        = LEAD_IN_S
_T_ROLL1        = _T_ROLL0 + NUM_ROLL_CYCLES * OSCILLATION_PERIOD_S
_T_PITCH0       = _T_ROLL1 + PAUSE_S
_T_PITCH1       = _T_PITCH0 + NUM_PITCH_CYCLES * OSCILLATION_PERIOD_S
ATT_TOTAL_S     = _T_PITCH1 + LEAD_OUT_S
ATT_MAX_DEG     = max(abs(ROLL_AMPLITUDE_DEG), abs(PITCH_AMPLITUDE_DEG))
ATT_PEAK_RATE_DPS = ATT_MAX_DEG * _OMEGA
if ATT_MAX_DEG > MAX_AMP_DEG:
    sys.exit(f"振幅超過 MAX_AMP_DEG = {MAX_AMP_DEG}°")

if PATH_SPEED_MPS > PX4_SPEED_XY_MAX:
    sys.exit(f"PATH_SPEED_MPS {PATH_SPEED_MPS} > _speed_xy_max {PX4_SPEED_XY_MAX} m/s")
# 前進距離（含前後各 SPEED_RAMP_S 的加減速，smoothstep 速度曲線的平均速度 = V/2）
RUN_S        = max(ATT_TOTAL_S, 2.0 * SPEED_RAMP_S)
PATH_LEN_M   = PATH_SPEED_MPS * (RUN_S - SPEED_RAMP_S)


# ─────────────────────────────────────────────────────────────
# Connect
# udpout 連線必須先送出封包對方才知道要回傳給誰，所以先送 heartbeat，之後由
# _sp_worker 每秒補送。只接受飛控本身的 heartbeat（排除 MAVProxy / QGC）。
# ─────────────────────────────────────────────────────────────
print(f"Connecting to PX4 ({CONNECTION})...")
master = mavutil.mavlink_connection(CONNECTION, source_system=255, source_component=191)


def _send_heartbeat():
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


_t_conn = time.time()
_last_hb = 0.0
while True:
    if time.time() - _last_hb > 1.0:
        _send_heartbeat()
        _last_hb = time.time()
    _m = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
    if (_m is not None and _m.type != mavutil.mavlink.MAV_TYPE_GCS
            and _m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
        master.target_system = _m.get_srcSystem()
        break
    if time.time() - _t_conn > 30.0:
        sys.exit("  ERROR: 30 s 內沒有收到飛控 heartbeat（檢查連線字串 / MAVProxy 輸出埠）")
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")
print(f"  Plan: roll ±{ROLL_AMPLITUDE_DEG:.0f}° × {NUM_ROLL_CYCLES} + {PAUSE_S:.0f} s level + "
      f"pitch ±{PITCH_AMPLITUDE_DEG:.0f}° × {NUM_PITCH_CYCLES}, period {OSCILLATION_PERIOD_S:.0f} s "
      f"(peak {ATT_PEAK_RATE_DPS:.0f}°/s)")
print(f"  Path: {PATH_SPEED_MPS} m/s, {RUN_S:.0f} s ≈ {PATH_LEN_M:.1f} m, "
      f"heading {'takeoff yaw' if PATH_YAW_DEG is None else f'{PATH_YAW_DEG:.0f}°'}  "
      f"(DEBUG_VECT @ {CMD_HZ:.0f} Hz, lead {CMD_LEAD_S:.2f} s)")

for _msg_id, _hz in _STREAMS_HZ.items():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(_msg_id), 1e6 / _hz, 0, 0, 0, 0, 0)
    time.sleep(0.05)


# ─────────────────────────────────────────────────────────────
# Telemetry state cache（只由 _rx_worker 寫入，主執行緒只讀）
# ─────────────────────────────────────────────────────────────
_ned    = {'x': 0.0, 'y': 0.0, 'z': -0.01, 't': 0.0}               # LOCAL_POSITION_NED
_att    = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 't': 0.0}        # ATTITUDE (NED/FRD)
_att_sp = {'roll': 0.0, 'pitch': 0.0, 't': 0.0}                    # ATTITUDE_TARGET（讓步後）
# SERVO_OUTPUT_RAW：實機 servos 在 port=1 (AUX)；SITL 全部在 port=0（ch4-11 = servos）
_act    = {'servos': [], 'sitl_detected': False}
_hb     = {'main_mode': None, 'armed': False, 't': 0.0}             # 飛控 HEARTBEAT
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
        _att['t'] = time.time()
    elif t == 'ATTITUDE_TARGET':
        w, x, y, z = msg.q
        _att_sp['roll']  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        _att_sp['pitch'] = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        _att_sp['t'] = time.time()
    elif t == 'HEARTBEAT':
        if (msg.get_srcSystem() != master.target_system
                or msg.type == mavutil.mavlink.MAV_TYPE_GCS
                or msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID):
            return
        main_mode = (msg.custom_mode >> 16) & 0xFF
        if _hb['main_mode'] is not None and main_mode != _hb['main_mode']:
            print(f"\n  [MODE] {_PX4_MODE_NAMES.get(_hb['main_mode'], _hb['main_mode'])}"
                  f" → {_PX4_MODE_NAMES.get(main_mode, main_mode)}"
                  f"  (setpoint 最大間隔 {_sp_stat['max_gap']*1000:.0f} ms)", flush=True)
        _hb['main_mode'] = main_mode
        _hb['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        _hb['t'] = time.time()
    elif t == 'PARAM_VALUE':
        with _param_cv:
            _params[msg.param_id.rstrip('\x00')] = (msg.param_value, time.time())
            _param_cv.notify_all()
    elif t == 'STATUSTEXT':
        print(f"\n  [PX4] {msg.text.rstrip(chr(0))}", flush=True)
    elif t == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"\n  [ARM] rejected (result={msg.result})，原因見上方 [PX4] 訊息", flush=True)
    elif t == 'SERVO_OUTPUT_RAW':
        ch = [msg.servo1_raw,  msg.servo2_raw,  msg.servo3_raw,  msg.servo4_raw,
              msg.servo5_raw,  msg.servo6_raw,  msg.servo7_raw,  msg.servo8_raw,
              msg.servo9_raw,  msg.servo10_raw, msg.servo11_raw, msg.servo12_raw]
        if msg.port == 0:
            if not _act['sitl_detected'] and any(1100 < v < 1900 for v in ch[4:12]):
                _act['sitl_detected'] = True
            if _act['sitl_detected']:
                _act['servos'] = ch[4:12]
        elif msg.port == 1:
            _act['sitl_detected'] = False   # 有 port=1 → 實機
            _act['servos'] = ch[:8]


def _rx_worker():
    """唯一讀取 socket 的執行緒；主執行緒的任何等待都不會卡住 setpoint 串流。"""
    while _running.is_set():
        try:
            msg = master.recv_match(blocking=True, timeout=0.1)
        except Exception as e:          # 單一壞封包不應終止接收執行緒
            print(f"\n  [RX] error: {e}", flush=True)
            continue
        if msg is not None and msg.get_type() != 'BAD_DATA':
            _handle_msg(msg)


# ─────────────────────────────────────────────────────────────
# Background setpoint thread（唯一發 setpoint 的執行緒，固定 SP_RATE_HZ）
# PX4 offboard_control_mode 超過 COM_OF_LOSS_T（SITL 0.5 s、實機 1 s）未更新
# → offboard signal lost → failsafe。
# ─────────────────────────────────────────────────────────────
_tx_lock   = threading.Lock()      # pymavlink 的 mav.send() 非 thread-safe（seq 計數）
_running   = threading.Event()
_running.set()
# 目前 setpoint (NED)：((x, y, z), (vx, vy, vz), (ax, ay, az), yaw)，單一 tuple 指派（原子）
_sp_pvay   = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
_SP_TYPE_MASK = 0b0000100000000000   # position + velocity + acceleration + yaw，忽略 yaw_rate
_sp_active = threading.Event()
_sp_active.set()
_sp_stat   = {'last': 0.0, 'max_gap': 0.0}


def _send(fn, *args):
    with _tx_lock:
        fn(*args)


def _sp_worker():
    period = 1.0 / SP_RATE_HZ
    next_t = time.monotonic()
    last_hb = 0.0
    while _running.is_set():
        now = time.monotonic()
        if now - last_hb >= 1.0:
            _send(_send_heartbeat)
            last_hb = now
        if _sp_active.is_set():
            (x, y, z), (vx, vy, vz), (ax, ay, az), yaw = _sp_pvay
            _send(master.mav.set_position_target_local_ned_send,
                  0, master.target_system, master.target_component,
                  mavutil.mavlink.MAV_FRAME_LOCAL_NED, _SP_TYPE_MASK,
                  x, y, z, vx, vy, vz, ax, ay, az, yaw, 0)
            if _sp_stat['last']:
                gap = now - _sp_stat['last']
                _sp_stat['max_gap'] = max(_sp_stat['max_gap'], gap)
                if gap > 0.3:
                    print(f"\n  [SP] WARNING: setpoint gap {gap*1000:.0f} ms", flush=True)
            _sp_stat['last'] = now
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

def set_pva(p, v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0), yaw=None):
    """更新 setpoint 位置 / 速度 / 加速度 / yaw (NED)；實際發送由 _sp_worker 負責。"""
    global _sp_pvay
    _sp_pvay = (tuple(p), tuple(v), tuple(a), _sp_pvay[3] if yaw is None else yaw)


def min_jerk(t: float, T: float):
    """最小 jerk 曲線 s(t)∈[0,1]：回傳 (s, ṡ, s̈)，起訖速度與加速度皆為 0。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    u = t / T
    return (u**3 * (10.0 - 15.0 * u + 6.0 * u * u),
            30.0 * u * u * (1.0 - u) ** 2 / T,
            60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T))


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_turn_duration(yaw0: float, yaw1: float) -> float:
    d = abs(wrap_pi(yaw1 - yaw0))
    return max(YAW_MIN_TURN_S, MIN_JERK_PEAK * d / math.radians(YAW_PEAK_RATE_DPS))


def _param_wait(name, t_sent, timeout):
    with _param_cv:
        got = _param_cv.wait_for(lambda: name in _params and _params[name][1] >= t_sent, timeout)
        return _params[name][0] if got else None


def param_set(name: str, value: float, retries: int = 5, timeout: float = 1.5) -> bool:
    """REAL32 參數：PARAM_SET + 等本次之後的 PARAM_VALUE 並驗證數值（setpoint 照常送出）。"""
    for _ in range(retries):
        t_sent = time.time()
        _send(master.mav.param_set_send, master.target_system, master.target_component,
              name.encode('utf-8'), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        actual = _param_wait(name, t_sent, timeout)
        if actual is not None and abs(actual - value) < 0.01:
            return True
    return False


def param_set_int(name: str, value: int, retries: int = 5, timeout: float = 1.5) -> bool:
    """INT32 參數（PX4 以 bytewise 方式傳：int 的位元放在 float 欄位裡）。"""
    raw = struct.unpack('<f', struct.pack('<i', int(value)))[0]
    for _ in range(retries):
        t_sent = time.time()
        _send(master.mav.param_set_send, master.target_system, master.target_component,
              name.encode('utf-8'), raw, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        actual = _param_wait(name, t_sent, timeout)
        if actual is not None and struct.unpack('<i', struct.pack('<f', actual))[0] == int(value):
            return True
    return False


def param_get(name: str, timeout: float = 3.0):
    t_sent = time.time()
    _send(master.mav.param_request_read_send, master.target_system, master.target_component,
          name.encode('utf-8'), -1)
    return _param_wait(name, t_sent, timeout)


def param_get_int(name: str, timeout: float = 3.0):
    v = param_get(name, timeout)
    return None if v is None else struct.unpack('<i', struct.pack('<f', v))[0]


_stream_t0 = time.time()   # DEBUG_VECT z 的時間基準（整個串流共用一個時鐘，角速度才連續）


def send_att_cmd(roll_deg: float, pitch_deg: float):
    """DEBUG_VECT "att_cmd"：x = roll°、y = pitch°、z = 取樣時間 (s，韌體用來算角速度)。"""
    _send(master.mav.debug_vect_send, b'att_cmd', int(time.time() * 1e6),
          float(roll_deg), float(pitch_deg), float(time.time() - _stream_t0))


def attitude_stream_off():
    """PFA_ATT_SRC = 0、PFA_DES_ROLL/PITCH = 0（等 ACK，setpoint 照常送出）。"""
    ok = param_set_int('PFA_ATT_SRC', 0)
    print(f"  PFA_ATT_SRC = 0 ... {'OK' if ok else 'FAILED'}")
    for name in ('PFA_DES_ROLL', 'PFA_DES_PITCH'):
        ok2 = param_set(name, 0.0)
        print(f"  {name} = 0 ... {'OK' if ok2 else 'FAILED'}")
        ok = ok and ok2
    if not ok:
        print("  ✗ 參數未全部設回，請在 QGC 確認 PFA_ATT_SRC = 0、PFA_DES_ROLL/PITCH = 0！"
              "（串流停止後韌體仍會逾時回到參數值）")


def wait_until(cond, timeout: float) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def wait_for_mode(target_main_mode: int, timeout: float = 5) -> bool:
    return wait_until(lambda: _hb['main_mode'] == target_main_mode, timeout)


def flight_ok() -> bool:
    return _hb['main_mode'] == 6 and _hb['armed']


def set_mode_offboard():
    _send(master.mav.command_long_send, master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 6.0, 0, 0, 0, 0, 0)


def set_mode_land():
    _send(master.mav.command_long_send, master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 4.0, 6.0, 0, 0, 0, 0)


def arm(force: bool = False):
    _send(master.mav.command_long_send, master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
          1.0, 21196.0 if force else 0.0, 0, 0, 0, 0, 0)


def shutdown(code: int):
    _sp_active.clear()
    _running.clear()
    sys.exit(code)


class Aborted(Exception):
    """離開 OFFBOARD / 被上鎖：交給飛手 / failsafe，不再切模式。"""


def check_flight():
    if not flight_ok():
        mode = _PX4_MODE_NAMES.get(_hb['main_mode'], _hb['main_mode'])
        raise Aborted(f"left OFFBOARD / disarmed (mode={mode}, armed={_hb['armed']})")


def hold(p, yaw, duration):
    set_pva(p, yaw=yaw)
    t_w = time.time()
    while time.time() - t_w < duration:
        check_flight()
        time.sleep(0.05)


def sample_attitude(duration: float):
    """取樣 duration 秒的實測 roll / pitch (deg)，並維持飛行檢查。
    回傳 (roll 平均, pitch 平均, roll σ, pitch σ)。"""
    rs, ps = [], []
    end = time.time() + duration
    while time.time() < end:
        check_flight()
        rs.append(math.degrees(_att['roll']))
        ps.append(math.degrees(_att['pitch']))
        time.sleep(0.05)
    mr, mp = sum(rs) / len(rs), sum(ps) / len(ps)
    sr = math.sqrt(sum((v - mr) ** 2 for v in rs) / len(rs))
    sp = math.sqrt(sum((v - mp) ** 2 for v in ps) / len(ps))
    return mr, mp, sr, sp


def stream_hold(p, yaw, roll_deg, pitch_deg, duration):
    """在 p 懸停，同時以 CMD_HZ 串流固定姿態指令（例如 −偏差）。"""
    set_pva(p, yaw=yaw)
    end = time.time() + duration
    while time.time() < end:
        check_flight()
        send_att_cmd(roll_deg, pitch_deg)
        time.sleep(1.0 / CMD_HZ)


def servo_max_deg():
    """所有舵機中偏轉最大的角度（±30° ↔ 1200–1800 µs）；無資料回傳 None。"""
    s = _act['servos']
    return max(abs(v - 1500) / 300.0 * 30.0 for v in s) if s else None


def tilt_deg(r_deg: float, p_deg: float) -> float:
    c = math.cos(math.radians(r_deg)) * math.cos(math.radians(p_deg))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _rms(x):
    return math.sqrt(sum(v * v for v in x) / len(x)) if x else float('nan')


# ─────────────────────────────────────────────────────────────
# Forward motion: distance s(t) along the path, smoothstep speed ramps
# ─────────────────────────────────────────────────────────────

def forward_profile(t: float):
    """(s, v, a)：沿路徑的距離 / 速度 / 加速度。前後 SPEED_RAMP_S 以 smoothstep 加減速。"""
    V, R, T = PATH_SPEED_MPS, SPEED_RAMP_S, RUN_S
    t = min(max(t, 0.0), T)
    if t < R:
        u = t / R
        return V * R * u**3 * (1.0 - 0.5 * u), V * u * u * (3.0 - 2.0 * u), V * 6.0 * u * (1.0 - u) / R
    s_end = V * (T - R)
    if t > T - R:
        r = (T - t) / R
        return s_end - V * R * r**3 * (1.0 - 0.5 * r), V * r * r * (3.0 - 2.0 * r), -V * 6.0 * r * (1.0 - r) / R
    return 0.5 * V * R + V * (t - R), V, 0.0


# ─────────────────────────────────────────────────────────────
# Attitude profile: (roll°, pitch°, segment) at time t of the run
# ─────────────────────────────────────────────────────────────
def attitude_cmd_deg(t: float):
    if _T_ROLL0 <= t < _T_ROLL1:
        return ROLL_AMPLITUDE_DEG * math.sin(_OMEGA * (t - _T_ROLL0)), 0.0, 'roll'
    if _T_PITCH0 <= t < _T_PITCH1:
        return 0.0, PITCH_AMPLITUDE_DEG * math.sin(_OMEGA * (t - _T_PITCH0)), 'pitch'
    return 0.0, 0.0, 'level'


# ─────────────────────────────────────────────────────────────
# Step 0 – 檢查韌體、PFA_ATT_SRC = 1
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] PFA_DES_ROLL/PITCH = 0°, PFA_ATT_SRC = 1 (DEBUG_VECT)")
if param_get('PFA_ATT_SRC') is None:
    print("  ERROR: FCU 沒有 PFA_ATT_SRC 參數 → 韌體不含 DEBUG_VECT 姿態串流，請更新 pfa_pos_control。")
    shutdown(1)
for _name in ('PFA_DES_ROLL', 'PFA_DES_PITCH'):
    print(f"  {_name} = 0 ... {'OK' if param_set(_name, 0.0) else 'FAILED'}")
for _name, _val in (('PFA_ATT_TOUT', ATT_TOUT_S), ('PFA_ATT_EXTR', ATT_EXTR_S), ('PFA_ATT_SLEW', ATT_SLEW_DPS)):
    if _val is not None:
        print(f"  {_name} = {_val} ... {'OK' if param_set(_name, _val) else 'FAILED'}")
if not param_set_int('PFA_ATT_SRC', 1):
    print("  ERROR: PFA_ATT_SRC = 1 設定失敗。")
    shutdown(1)
print("  PFA_ATT_SRC = 1 ... OK")
_slew = param_get('PFA_ATT_SLEW')
_tilt = param_get('PFA_YIELD_TILT')
if _slew is not None and _slew <= ATT_PEAK_RATE_DPS:
    print(f"  WARNING: PFA_ATT_SLEW {_slew:.0f}°/s ≤ 指令峰值角速度 {ATT_PEAK_RATE_DPS:.0f}°/s，"
          f"串流開始 / 恢復時指令可能追不上")
if _tilt is not None and ATT_MAX_DEG > _tilt:
    print(f"  NOTE: 振幅 {ATT_MAX_DEG:.0f}° > PFA_YIELD_TILT {_tilt:.0f}° → 姿態讓步會把 setpoint "
          f"限制在約 {_tilt:.0f}°，以保持位置與高度")

print("\n[Step 1] Checking local position / attitude...")
if not wait_until(lambda: _ned['t'] > 0.0 and _att['t'] > 0.0, 5.0):
    print("  ERROR: No LOCAL_POSITION_NED / ATTITUDE received. Is EKF2 running?")
    attitude_stream_off()
    shutdown(1)
ground     = (_ned['x'], _ned['y'], _ned['z'])
ground_yaw = _att['yaw']
path_yaw   = ground_yaw if PATH_YAW_DEG is None else math.radians(PATH_YAW_DEG)
print(f"  OK – NED position: ({ground[0]:.2f}, {ground[1]:.2f}, {ground[2]:.2f})  "
      f"yaw={math.degrees(ground_yaw):.1f}°  → path heading {math.degrees(path_yaw):.1f}°")

tkf_byp = param_get_int('PFA_TKF_BYP')
spool_s = param_get('COM_SPOOLUP_TIME')
ramp_s  = param_get('MPC_TKO_RAMP_T')
if tkf_byp == 0:
    print("  WARNING: PFA_TKF_BYP = 0 → OFFBOARD 下收不到 want_takeoff，推力上限會卡在 0.1，請改回 1。")
spool_s = 1.0 if spool_s is None else spool_s
ramp_s  = 3.0 if ramp_s  is None else ramp_s
TAKEOFF_WAIT_S = spool_s + ramp_s + TAKEOFF_SETTLE_S
print(f"  Takeoff ramp: wait {TAKEOFF_WAIT_S:.1f} s after arming before climbing")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stream setpoints → OFFBOARD → Arm（解鎖前 setpoint = 地面位置與航向）
# ─────────────────────────────────────────────────────────────
print("\n[Step 2] Streaming ground-position setpoints (5 s)...")
set_pva(ground, yaw=ground_yaw)
time.sleep(5.0)

print("  Requesting OFFBOARD mode...")
set_mode_offboard()
if not wait_for_mode(6):
    time.sleep(2.5)
    set_mode_offboard()
print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: OFFBOARD not confirmed.")

print("  Arming...")
time.sleep(1.0)
arm()
if not wait_until(lambda: _hb['armed'], 5.0):
    arm(force=True)
    if not wait_until(lambda: _hb['armed'], 5.0):
        print("  ERROR: Failed to arm（原因見上方 [PX4] / [ARM] 訊息）.")
        attitude_stream_off()
        shutdown(1)
print("  Armed.")

stream_on = False
hover_p   = (ground[0], ground[1], -TARGET_ALT_M)
end_p     = hover_p
try:
    # ─────────────────────────────────────────────────────────
    # Step 3 – 平滑起飛：等 PX4 起飛緩升 → 最小 jerk 爬升 → 等高度穩定 → 轉向
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 3a] Holding ground position for {TAKEOFF_WAIT_S:.1f} s (PX4 takeoff ramp)...")
    hold(ground, ground_yaw, TAKEOFF_WAIT_S)

    z0   = _ned['z']
    dz   = -TARGET_ALT_M - z0
    T_cl = max(CLIMB_MIN_S, MIN_JERK_PEAK * abs(dz) / CLIMB_PEAK_MPS)
    print(f"\n[Step 3b] Smooth climb {-z0:.2f} m → {TARGET_ALT_M:.2f} m in {T_cl:.1f} s")
    t_cl0 = time.time()
    while True:
        check_flight()
        t = time.time() - t_cl0
        s_, sd, sdd = min_jerk(t, T_cl)
        set_pva((ground[0], ground[1], z0 + s_ * dz), (0.0, 0.0, sd * dz), (0.0, 0.0, sdd * dz), ground_yaw)
        if t >= T_cl:
            break
        time.sleep(0.05)
    set_pva(hover_p, yaw=ground_yaw)

    print(f"\n[Step 3c] Waiting for altitude (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")
    stable_since = None
    t_ph = time.time()
    while True:
        check_flight()
        now = time.time()
        stable_since = (stable_since or now) if abs(TARGET_ALT_M + _ned['z']) < ALT_TOL else None
        if stable_since and now - stable_since >= HOVER_STABLE_TIME:
            print(f"  Stable at {-_ned['z']:.2f} m.")
            break
        if now - t_ph > ALT_WAIT_TIMEOUT:
            print(f"  Timeout – alt={-_ned['z']:.2f} m, continuing.")
            break
        time.sleep(0.05)

    d_yaw = wrap_pi(path_yaw - ground_yaw)
    if abs(d_yaw) > math.radians(1.0):
        T_y = yaw_turn_duration(ground_yaw, path_yaw)
        print(f"\n[Step 3d] Turning {math.degrees(ground_yaw):.0f}° → {math.degrees(path_yaw):.0f}° in {T_y:.1f} s")
        t_y0 = time.time()
        while True:
            check_flight()
            t = time.time() - t_y0
            set_pva(hover_p, yaw=wrap_pi(ground_yaw + min_jerk(t, T_y)[0] * d_yaw))
            if t >= T_y:
                break
            time.sleep(0.05)
    print(f"\n[Step 4] Level hover for {HOVER_BEFORE_S:.0f} s...")
    hold(hover_p, path_yaw, HOVER_BEFORE_S)

    bias_r = bias_p = 0.0
    if BIAS_COMP:
        print(f"\n[Step 4b] Measuring attitude bias for {BIAS_MEASURE_S:.0f} s (command 0°, no stream)...")
        mr, mp, sr, sp = sample_attitude(BIAS_MEASURE_S)
        print(f"  measured roll {mr:+.1f}° (σ {sr:.1f}°)   pitch {mp:+.1f}° (σ {sp:.1f}°)")
        bias_r = max(-BIAS_MAX_DEG, min(BIAS_MAX_DEG, mr))
        bias_p = max(-BIAS_MAX_DEG, min(BIAS_MAX_DEG, mp))
        if (bias_r, bias_p) != (mr, mp):
            print(f"  WARNING: bias exceeds ±{BIAS_MAX_DEG:.0f}° → clipped to roll {bias_r:+.1f}°, "
                  f"pitch {bias_p:+.1f}°（請檢查重心 / 電纜）")
        print(f"\n[Step 4c] Bias compensation: stream roll {-bias_r:+.1f}°, pitch {-bias_p:+.1f}° "
              f"for {BIAS_SETTLE_S:.0f} s")
        stream_on = True
        _stream_t0 = time.time()
        stream_hold(hover_p, path_yaw, -bias_r, -bias_p, max(0.0, BIAS_SETTLE_S - 2.0) + 0.5)
        cr, cp = 0.0, 0.0
        t_v = time.time()
        rs, ps = [], []
        while time.time() - t_v < 2.0:
            check_flight()
            send_att_cmd(-bias_r, -bias_p)
            rs.append(math.degrees(_att['roll'])); ps.append(math.degrees(_att['pitch']))
            time.sleep(1.0 / CMD_HZ)
        cr, cp = sum(rs) / len(rs), sum(ps) / len(ps)
        print(f"  after compensation: roll {cr:+.1f}°  pitch {cp:+.1f}°  (目標 0°)")

    # ─────────────────────────────────────────────────────────
    # Step 5 – 直線前進 + 姿態串流
    # ─────────────────────────────────────────────────────────
    cdir, sdir = math.cos(path_yaw), math.sin(path_yaw)
    print(f"\n[Step 5] {RUN_S:.0f} s forward + attitude stream (DEBUG_VECT)")
    print(f"  {'t':>5}  {'seg':<6} {'roll r/sp/m':>17}  {'pitch r/sp/m':>17}  "
          f"{'s_cmd':>5} {'along':>6} {'cross':>6} {'alt':>5} {'servo':>5}"
          f"   (r = 參考(t)，送出 r(t + {CMD_LEAD_S:.2f} s) − b，b = ({bias_r:+.1f}°, {bias_p:+.1f}°))")
    stats = {}        # segment → dict of lists
    n_sent = 0
    cmd_dt = 1.0 / CMD_HZ
    t0 = time.time()
    if not stream_on:
        _stream_t0 = t0
    stream_on = True
    last_print = 0.0
    next_tick = 0.0
    while True:
        check_flight()
        t = time.time() - t0
        if t >= RUN_S:
            break
        s_, v_, a_ = forward_profile(t)
        p = (hover_p[0] + s_ * cdir, hover_p[1] + s_ * sdir, -TARGET_ALT_M)
        set_pva(p, (v_ * cdir, v_ * sdir, 0.0), (a_ * cdir, a_ * sdir, 0.0), path_yaw)

        # 值 = 參考(t + CMD_LEAD_S) − 偏差；z = 串流時鐘（韌體以 Δ值/Δz 算角速度，兩者一致）
        r_lead, p_lead, _ = attitude_cmd_deg(t + CMD_LEAD_S)
        send_att_cmd(r_lead - bias_r, p_lead - bias_p)
        n_sent += 1

        r_c, p_c, seg = attitude_cmd_deg(t)
        r_s, p_s = math.degrees(_att_sp['roll']), math.degrees(_att_sp['pitch'])
        r_m, p_m = math.degrees(_att['roll']), math.degrees(_att['pitch'])
        ex, ey = _ned['x'] - p[0], _ned['y'] - p[1]
        along, cross = ex * cdir + ey * sdir, -ex * sdir + ey * cdir
        if seg != 'level':
            st = stats.setdefault(seg, {k: [] for k in ('er', 'ep', 'esr', 'esp', 'cross', 'along', 'alt', 'tilt_sp')})
            st['er'].append(r_c - r_m); st['ep'].append(p_c - p_m)
            st['esr'].append(r_s - r_m); st['esp'].append(p_s - p_m)
            st['cross'].append(cross); st['along'].append(along)
            st['alt'].append(-_ned['z']); st['tilt_sp'].append(tilt_deg(r_s, p_s))

        now = time.time()
        if now - last_print >= LOG_PERIOD_S:
            sv = servo_max_deg()
            print(f"  {t:5.1f}  {seg:<6} {r_c:+5.1f}/{r_s:+5.1f}/{r_m:+5.1f}  {p_c:+5.1f}/{p_s:+5.1f}/{p_m:+5.1f}  "
                  f"{s_:5.2f} {along:+6.2f} {cross:+6.2f} {-_ned['z']:5.2f} "
                  f"{(f'{sv:4.1f}°' if sv is not None else '  n/a')}")
            last_print = now

        next_tick += cmd_dt
        wait = next_tick - (time.time() - t0)
        if wait > 0:
            time.sleep(wait)
        else:
            next_tick = time.time() - t0   # 落後就不補送，從現在重新排程
    elapsed = time.time() - t0
    end_p = (hover_p[0] + PATH_LEN_M * cdir, hover_p[1] + PATH_LEN_M * sdir, -TARGET_ALT_M)
    set_pva(end_p, yaw=path_yaw)
    print(f"\n  DEBUG_VECT: {n_sent} sent in {elapsed:.1f} s (≈ {n_sent / max(elapsed, 1e-3):.1f} Hz), "
          f"setpoint max gap {_sp_stat['max_gap']*1000:.0f} ms")

    for seg, st in stats.items():
        print(f"  [{seg}] ref−meas: roll RMS {_rms(st['er']):.1f}° pitch RMS {_rms(st['ep']):.1f}°  |  "
              f"sp−meas: roll {_rms(st['esr']):.1f}° pitch {_rms(st['esp']):.1f}°  |  "
              f"sp tilt max {max(st['tilt_sp']):.1f}°")
        print(f"  [{seg}] cross-track RMS {_rms(st['cross']):.2f} m (max {max(map(abs, st['cross'])):.2f})  "
              f"along-track RMS {_rms(st['along']):.2f} m  alt {min(st['alt']):.2f}–{max(st['alt']):.2f} m")

    # ─────────────────────────────────────────────────────────
    # Step 6 – 終點懸停
    # ─────────────────────────────────────────────────────────
    # 終點懸停：串流維持 −偏差（rate = 0，韌體停在補償值而不是外插），降落前才關串流
    print(f"\n[Step 6] Holding end point for {HOLD_END_S:.0f} s (stream {-bias_r:+.1f}°, {-bias_p:+.1f}°)...")
    stream_hold(end_p, path_yaw, -bias_r, -bias_p, HOLD_END_S)
    attitude_stream_off()
    stream_on = False

except Aborted as e:
    print(f"\n  ✗ Aborted: {e} → 不再切換模式，交給飛手 / failsafe。")
    if stream_on:
        attitude_stream_off()
    shutdown(1)
except KeyboardInterrupt:
    print("\n  Ctrl-C → 姿態歸零、原地懸停 → AUTO.LAND")
    end_p = (_ned['x'], _ned['y'], -TARGET_ALT_M)
    set_pva(end_p, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    if stream_on:
        b_r, b_p = globals().get('bias_r', 0.0), globals().get('bias_p', 0.0)
        for _ in range(int(0.3 * CMD_HZ)):
            send_att_cmd(-b_r, -b_p)
            time.sleep(1.0 / CMD_HZ)
        attitude_stream_off()
    time.sleep(2.0)

# ─────────────────────────────────────────────────────────────
# Land
# ─────────────────────────────────────────────────────────────
print("\n[Land] Landing (AUTO.LAND)...")
set_mode_land()
# 確認已離開 OFFBOARD 才停止 setpoint，否則會先觸發 offboard lost → failsafe（RTL）
if not wait_for_mode(4, 5.0):
    print("  WARNING: AUTO.LAND not confirmed, retrying...")
    set_mode_land()
    wait_for_mode(4, 5.0)
_sp_active.clear()
print("  讓 AUTO.LAND 自行偵測落地後上鎖，請勿在空中切 RC 上鎖開關。")
time.sleep(15)
_running.clear()
print("Done.")
