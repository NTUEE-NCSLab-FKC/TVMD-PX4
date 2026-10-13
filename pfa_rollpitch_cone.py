#!/usr/bin/env python3
"""
PFA Roll + Pitch Cone (Coning) Tracking (pymavlink)
====================================================
pfa_rollpitch_sine.py 的 90° 相位版本（MAVROS 版：pfa_rollpitch_cone_mavros.py）：
原地懸停，roll 與 pitch 以相差 90° 的正弦波擺動（預設 ±20°），機身傾斜方向繞圈
（錐形擺動），位置維持不動。

■ 姿態指令怎麼送進 PX4
  pfa_pos_control 的期望 roll / pitch 只來自參數 PFA_DES_ROLL / PFA_DES_PITCH
  （yaw 來自 trajectory_setpoint），所以正弦波是以 PARAM_HZ 的頻率連續送
  PARAM_SET 這兩個參數：
    θ_roll(t)  = ROLL_AMP_DEG  · e(t) · sin(ωt)
    θ_pitch(t) = PITCH_AMP_DEG · e(t) · sin(ωt + PITCH_PHASE_DEG)
    ω = 2π / SINE_PERIOD_S
  e(t) 為振幅包絡：前 ENV_RAMP_S 以最小 jerk 曲線 0 → 1，中間 NUM_CYCLES 個
  完整週期 e = 1，最後 ENV_RAMP_S 再 1 → 0。避免一開始角速度就跳到 A·ω。
  本版固定 PITCH_PHASE_DEG = ±90° → 錐形擺動（coning）：
    θ_roll = A·sin(ωt)、θ_pitch = ±A·cos(ωt)，一軸最大時另一軸為 0，
    機頂傾斜方向以週期 SINE_PERIOD_S 繞圈，合成傾角大致維持 A（±20° → 約 20°），
    機身頂部畫出一個圓錐；yaw 不變（機頭方向固定）。
    CONE_CCW = True  → 俯視逆時針（機頂依序傾向 後 → 右 → 前 → 左）
    CONE_CCW = False → 俯視順時針（後 → 左 → 前 → 右）
  與同相版（PITCH_PHASE_DEG = 0）相比，合成傾角沒有 28° 的尖峰，但每一刻都維持
  約 A 的傾角，舵機持續處於大偏轉；結束時會額外統計實際合成傾角。

■ 與 MAVROS 版的差異
  - 擺動期間 PARAM_SET 不等 ACK（MAVROS 的 /mavros/param/set 一定會等），
    主迴圈可以穩定維持 PARAM_HZ；FCU 回傳的 PARAM_VALUE 由 _rx_worker 收進快取，
    用來顯示 FCU 目前的實際參數值與統計 ACK 數。掉包時下一拍會送新值，不需重送。
    起始 / 結束歸零則用會等 ACK 並驗證數值的 param_set()。
  - 姿態量測直接用 ATTITUDE（NED/FRD），與 PX4 正負號相同，不需轉換。
  - setpoint 帶 yaw = 解鎖前的航向（type_mask 只忽略 yaw_rate）。若忽略 yaw，
    pfa_pos_control 會把期望 yaw 設成「目前 yaw」，等於完全不控制 yaw。

■ 執行緒架構（與 pfa_pitch_tracking_smooth.py 相同）
  _rx_worker  — 唯一讀 socket 的執行緒，更新遙測快取 / PARAM_VALUE / HEARTBEAT
  _sp_worker  — 唯一發 setpoint 的執行緒，固定 SP_RATE_HZ
  主執行緒    — 只讀快取、改 setpoint、送 PARAM_SET；任何等待都不會中斷 setpoint
  所有 send 都經過 _tx_lock（pymavlink 的 mav.send() 非 thread-safe）。

■ 數傳頻寬（57600 baud，MAV_0_RATE ≈ 1200 B/s）
  每拍兩個 PARAM_SET（約 2×35 B 上行）與兩個 PARAM_VALUE（約 2×37 B 下行），
  PARAM_HZ = 5 時下行約 370 B/s，加上 ATTITUDE 10 Hz、LOCAL_POSITION_NED 5 Hz
  仍在預算內；不要把 PARAM_HZ 拉太高。每次改參數 PX4 也會觸發 parameter_update
  與自動存檔。

■ 安全
  - 期間若離開 OFFBOARD 或被上鎖，立即中止並把 PFA_DES_ROLL/PITCH 設回 0
    （這兩個參數在其他飛行模式下也會被 pfa_pos_control 使用），之後不再切換
    模式，交給飛手 / failsafe 處理
  - 任何例外 / Ctrl-C 也會先把兩個參數設回 0 再離開
  - 振幅上限 MAX_AMP_DEG；roll、pitch 各軸需要的機身側向推力約
    T_hover·sin(θ)，必須小於 pfa_pos_control 的 _thrust_xy_max = 0.3

■ 起飛
  與 pfa_pitch_tracking_smooth.py 相同的平滑起飛：解鎖前 setpoint = 地面位置，
  等 PX4 起飛緩升（COM_SPOOLUP_TIME + MPC_TKO_RAMP_T）結束後以最小 jerk 曲線爬升。
  OFFBOARD 下必須 PFA_TKF_BYP = 1（啟動時會檢查）。

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
CONNECTION        = 'udp:127.0.0.1:14550'

TARGET_ALT_M      = 1.0    # 懸停高度 (m)；光流 SENS_FLOW_MINHGT = 0.8 m，建議 ≥ 1.0

ROLL_AMP_DEG      = 20.0   # roll 振幅 (deg)，正值 = 右翼朝下
PITCH_AMP_DEG     = 20.0   # pitch 振幅 (deg)，正值 = 機頭朝上
SINE_PERIOD_S     = 10.0   # 正弦週期 (s)；峰值角速度 = 2π·A/T（20°, 10 s → 12.6°/s）
CONE_CCW          = True   # True: 俯視逆時針（後→右→前→左）; False: 順時針
PITCH_PHASE_DEG   = 90.0 if CONE_CCW else -90.0   # pitch 相對 roll 的相位 (deg)
NUM_CYCLES        = 3      # 滿振幅的完整週期數
ENV_RAMP_S        = SINE_PERIOD_S   # 振幅包絡淡入 / 淡出時間 (s)
MAX_AMP_DEG       = 30.0   # 振幅硬上限 (deg)

PARAM_HZ          = 5.0    # PFA_DES_ROLL/PITCH 更新頻率 (Hz)
PARAM_DEADBAND_DEG = 0.2   # 與上次送出值差異小於此值就不送

HOVER_BEFORE_S    = 5.0    # 開始擺動前的懸停時間 (s)
HOVER_AFTER_S     = 5.0    # 擺動結束、姿態歸零後的懸停時間 (s)
ALT_TOL           = 0.25   # 高度穩定容許誤差 (m)
HOVER_STABLE_TIME = 2.0    # 高度在容許範圍內需持續的時間 (s)
ALT_WAIT_TIMEOUT  = 15.0   # 等待高度穩定的上限 (s)

# 平滑起飛
CLIMB_PEAK_MPS    = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S       = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S  = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × 距離 / 時間

# offboard setpoint 串流頻率 (Hz)。PX4 只要求間隔 < COM_OF_LOSS_T (1 s)；
# 經 57600 baud 數傳時每筆 65 B，10 Hz ≈ 650 B/s 已足夠且不壅塞鏈路
SP_RATE_HZ        = 10.0
LOG_PERIOD_S      = 0.5    # 追蹤狀態印出間隔 (s)

# 遙測串流 (msg id → Hz)：ATTITUDE 10 Hz（姿態追蹤）、LOCAL_POSITION_NED 5 Hz、
# SERVO_OUTPUT_RAW 2 Hz（舵機角度監看）
_STREAMS_HZ = {30: 10.0, 32: 5.0, 36: 2.0}

_OMEGA     = 2.0 * math.pi / SINE_PERIOD_S
_PHASE_RAD = math.radians(PITCH_PHASE_DEG)
_FULL_S    = NUM_CYCLES * SINE_PERIOD_S
_TOTAL_S   = 2.0 * ENV_RAMP_S + _FULL_S

if max(abs(ROLL_AMP_DEG), abs(PITCH_AMP_DEG)) > MAX_AMP_DEG:
    sys.exit(f"ROLL/PITCH_AMP_DEG 超過 MAX_AMP_DEG = {MAX_AMP_DEG}°")

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print(f"Connecting to PX4 ({CONNECTION})...")
master = mavutil.mavlink_connection(CONNECTION)
master.wait_heartbeat()
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")
print(f"  Plan (cone, {'CCW' if CONE_CCW else 'CW'}): alt={TARGET_ALT_M} m  roll ±{ROLL_AMP_DEG}°  pitch ±{PITCH_AMP_DEG}°  "
      f"period={SINE_PERIOD_S} s  phase={PITCH_PHASE_DEG}°  "
      f"{ENV_RAMP_S:.0f} s fade-in + {NUM_CYCLES} cycles + {ENV_RAMP_S:.0f} s fade-out "
      f"= {_TOTAL_S:.0f} s  (PARAM_SET @ {PARAM_HZ} Hz)")
print(f"  Peak rate: roll {math.degrees(_OMEGA) * math.radians(ROLL_AMP_DEG):.1f}°/s  "
      f"pitch {math.degrees(_OMEGA) * math.radians(PITCH_AMP_DEG):.1f}°/s")

# PX4 在 GCS link 預設不一定送這些訊息（或頻率很低），用 MAV_CMD_SET_MESSAGE_INTERVAL 訂閱
for _msg_id, _hz in _STREAMS_HZ.items():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(_msg_id), 1e6 / _hz,
        0, 0, 0, 0, 0)
    time.sleep(0.05)
print("  Requested ATTITUDE 10 Hz + LOCAL_POSITION_NED 5 Hz + SERVO_OUTPUT_RAW 2 Hz")


# ─────────────────────────────────────────────────────────────
# Telemetry state cache（只由 _rx_worker 寫入，主執行緒只讀）
# ─────────────────────────────────────────────────────────────
_ned    = {'x': 0.0, 'y': 0.0, 'z': -0.01, 't': 0.0}   # LOCAL_POSITION_NED
_att    = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 't': 0.0}  # ATTITUDE (NED/FRD)
# SERVO_OUTPUT_RAW：實機 servos 在 port=1 (AUX)；SITL 全部在 port=0（ch4-11 = servos）
_act    = {'servos': [], 'sitl_detected': False}
_hb     = {'main_mode': None, 'sub_mode': None, 'armed': False, 't': 0.0}  # 飛控 HEARTBEAT
_params   = {}                         # PARAM_VALUE: name → (value, recv_time)
_param_cv = threading.Condition()
_param_acks = {'PFA_DES_ROLL': 0, 'PFA_DES_PITCH': 0}  # 擺動期間收到的 PARAM_VALUE 數

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
        name = msg.param_id.rstrip('\x00')
        with _param_cv:
            _params[name] = (msg.param_value, time.time())
            if name in _param_acks:
                _param_acks[name] += 1
            _param_cv.notify_all()
    elif t == 'STATUSTEXT':
        print(f"\n  [PX4] {msg.text.rstrip(chr(0))}", flush=True)
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


def servo_max_deg():
    """所有舵機中偏轉最大的角度（±30° ↔ 1200–1800 µs）；無資料回傳 None。"""
    s = _act['servos']
    if not s:
        return None
    return max(abs(v - 1500) / 300.0 * 30.0 for v in s)


# ─────────────────────────────────────────────────────────────
# Background setpoint thread
# 唯一負責 offboard setpoint 的執行緒，固定 SP_RATE_HZ 發送，與主迴圈完全解耦。
# PX4 offboardCheck：offboard_control_mode 超過 COM_OF_LOSS_T（預設 1 s）未更新
# 即判定 offboard signal lost → failsafe 切到 Position mode。
# ─────────────────────────────────────────────────────────────
_tx_lock   = threading.Lock()      # pymavlink 的 mav.send() 非 thread-safe（seq 計數）
_running   = threading.Event()     # rx / setpoint 執行緒存活旗標
_running.set()
# 目前 setpoint (NED)：((x, y, z), (vx, vy, vz), (ax, ay, az), yaw)。
# 主執行緒以單一 tuple 指派更新（原子操作），_sp_worker 每次讀一份完整快照。
_sp_pvay   = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
# position + velocity + acceleration + yaw，只忽略 yaw_rate (bit 11)
_SP_TYPE_MASK = 0b0000100000000000
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
            (x, y, z), (vx, vy, vz), (ax, ay, az), yaw = _sp_pvay
            _send(master.mav.set_position_target_local_ned_send,
                  0, master.target_system, master.target_component,
                  mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                  _SP_TYPE_MASK,
                  x, y, z,
                  vx, vy, vz,
                  ax, ay, az,
                  yaw, 0)
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
# MAVLink helpers（皆不讀 socket，只讀 _rx_worker 的快取）
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


def param_set(name: str, value: float, retries: int = 5, timeout: float = 1.5) -> bool:
    """PARAM_SET + 等待 PARAM_VALUE ACK，並驗證 FCU 回傳值與設定值一致。
    ACK 由 _rx_worker 接收，等待期間 setpoint 串流不受影響。"""
    nb = name.encode('utf-8')
    for attempt in range(retries):
        t_sent = time.time()
        _send(master.mav.param_set_send,
              master.target_system, master.target_component,
              nb, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        with _param_cv:
            got = _param_cv.wait_for(
                lambda: name in _params and _params[name][1] >= t_sent, timeout)
            actual = _params[name][0] if got else None
        if got:
            if abs(actual - value) < 0.01:
                return True
            print(f"    FCU ACK mismatch: {name} = {actual:.4f}  (sent {value:.4f})"
                  f"  retry {attempt+1}/{retries}")
    return False


def param_set_nowait(name: str, value: float):
    """只送 PARAM_SET，不等 ACK（擺動期間用；PARAM_VALUE 由 _rx_worker 收進快取）。"""
    _send(master.mav.param_set_send,
          master.target_system, master.target_component,
          name.encode('utf-8'), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def param_get(name: str, timeout: float = 3.0):
    t_sent = time.time()
    _send(master.mav.param_request_read_send,
          master.target_system, master.target_component,
          name.encode('utf-8'), -1)
    with _param_cv:
        if _param_cv.wait_for(
                lambda: name in _params and _params[name][1] >= t_sent, timeout):
            return _params[name][0]
    return None


def fcu_param(name: str, default: float = float('nan')) -> float:
    """快取中 FCU 最後回報的參數值。"""
    with _param_cv:
        return _params[name][0] if name in _params else default


def zero_attitude_params():
    """PFA_DES_ROLL/PITCH 設回 0（等 ACK 並驗證）。"""
    for name in ('PFA_DES_ROLL', 'PFA_DES_PITCH'):
        ok = param_set(name, 0.0)
        print(f"  {name} = 0 ... {'OK' if ok else 'FAILED'}")
        if not ok:
            print(f"  ✗ {name} 無法設回 0，請立即在 QGC 手動設定！")


def wait_until(cond, timeout: float) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def wait_for_mode(target_main_mode: int, timeout: float = 5) -> bool:
    return wait_until(lambda: _hb['main_mode'] == target_main_mode, timeout)


def wait_for_armed(timeout: float = 5) -> bool:
    return wait_until(lambda: _hb['armed'], timeout)


def flight_ok() -> bool:
    return _hb['main_mode'] == 6 and _hb['armed']


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


def arm(force: bool = False):
    _send(master.mav.command_long_send,
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
          1.0, 21196.0 if force else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def shutdown(code: int):
    _sp_active.clear()
    _running.clear()
    sys.exit(code)


# ─────────────────────────────────────────────────────────────
# Sine profile
# ─────────────────────────────────────────────────────────────
def envelope(t: float) -> float:
    """振幅包絡：淡入（最小 jerk）→ 1 → 淡出（最小 jerk）。"""
    if t < ENV_RAMP_S:
        return min_jerk(t, ENV_RAMP_S)[0]
    if t < ENV_RAMP_S + _FULL_S:
        return 1.0
    return 1.0 - min_jerk(t - ENV_RAMP_S - _FULL_S, ENV_RAMP_S)[0]


def attitude_cmd_deg(t: float):
    """t 秒時的 (roll, pitch) 指令 (deg，PX4 慣例)。"""
    e = envelope(t)
    return (ROLL_AMP_DEG  * e * math.sin(_OMEGA * t),
            PITCH_AMP_DEG * e * math.sin(_OMEGA * t + _PHASE_RAD))


# ─────────────────────────────────────────────────────────────
# Step 0 – PFA 姿態參數歸零、讀取起飛緩升與推力參數
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
zero_attitude_params()

print("\n[Step 1] Checking local position / attitude...")
if not wait_until(lambda: _ned['t'] > 0.0 and _att['t'] > 0.0, 5.0):
    print("  ERROR: No LOCAL_POSITION_NED / ATTITUDE received. Is EKF2 running?")
    shutdown(1)
ground     = (_ned['x'], _ned['y'], _ned['z'])
ground_yaw = _att['yaw']
print(f"  OK – NED position: ({ground[0]:.2f}, {ground[1]:.2f}, {ground[2]:.2f})  "
      f"yaw={math.degrees(ground_yaw):.1f}°")

tkf_byp = param_get('PFA_TKF_BYP')
spool_s = param_get('COM_SPOOLUP_TIME')
ramp_s  = param_get('MPC_TKO_RAMP_T')
mass    = param_get('PFA_VEH_MASS')
f_max   = param_get('PFA_MAX_THR')
if tkf_byp is not None and int(round(tkf_byp)) == 0:
    print("  WARNING: PFA_TKF_BYP = 0 → OFFBOARD 下收不到 want_takeoff，推力上限會卡在 0.1，"
          "請改回 1。")
spool_s = 1.0 if spool_s is None else spool_s
ramp_s  = 3.0 if ramp_s  is None else ramp_s
TAKEOFF_WAIT_S = spool_s + ramp_s + TAKEOFF_SETTLE_S
print(f"  Takeoff ramp: COM_SPOOLUP_TIME={spool_s:.1f} s + MPC_TKO_RAMP_T={ramp_s:.1f} s "
      f"→ wait {TAKEOFF_WAIT_S:.1f} s after arming before climbing")

# 各軸機身側向推力 ≈ T_hover·sin(θ)，需小於 _thrust_xy_max = 0.3
if mass and f_max:
    t_hover = mass * 9.81 / f_max
    worst   = max(abs(ROLL_AMP_DEG), abs(PITCH_AMP_DEG))
    need    = t_hover * math.sin(math.radians(worst))
    print(f"  Hover thrust (PFA_VEH_MASS/PFA_MAX_THR) ≈ {t_hover:.2f} → "
          f"lateral thrust at {worst:.0f}° ≈ {need:.2f} (limit 0.30)")
    if need > 0.25:
        print("  WARNING: 側向推力接近 _thrust_xy_max 0.3，位置可能保持不住；"
              "若實際懸停推力比參數大（PFA_MAX_THR 偏高），餘裕會更小。")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
# 解鎖前 setpoint = 目前地面位置與航向（高度誤差 0），而不是 TARGET_ALT_M
print(f"\n[Step 2] Streaming ground-position setpoints (5 s)...")
set_pva(ground, yaw=ground_yaw)
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
armed = wait_for_armed(5.0)
if not armed:
    arm(force=True)
    armed = wait_for_armed(5.0)
if not armed:
    print("  ERROR: Failed to arm.")
    shutdown(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – 平滑起飛（等 PX4 起飛緩升 → 最小 jerk 爬升）
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3a] Holding ground position for {TAKEOFF_WAIT_S:.1f} s (PX4 takeoff ramp)...")
time.sleep(TAKEOFF_WAIT_S)

z0   = _ned['z']                              # 從目前高度開始（NED，向下為正）
dz   = -TARGET_ALT_M - z0                     # NED 位移（爬升為負）
T_cl = max(CLIMB_MIN_S, MIN_JERK_PEAK * abs(dz) / CLIMB_PEAK_MPS)
print(f"\n[Step 3b] Smooth climb {-z0:.2f} m → {TARGET_ALT_M:.2f} m in {T_cl:.1f} s "
      f"(feedforward: p + v + a)")
t_cl0      = time.time()
last_print = 0.0
while True:
    t = time.time() - t_cl0
    s_, sd, sdd = min_jerk(t, T_cl)
    set_pva((ground[0], ground[1], z0 + s_ * dz), (0.0, 0.0, sd * dz), (0.0, 0.0, sdd * dz))
    now = time.time()
    if now - last_print > LOG_PERIOD_S:
        print(f"    {t:4.1f}s  alt_cmd={-(z0 + s_ * dz):.2f} m  alt={-_ned['z']:.2f} m")
        last_print = now
    if t >= T_cl:
        break
    time.sleep(0.05)
hover_p = (ground[0], ground[1], -TARGET_ALT_M)
set_pva(hover_p)   # 爬升結束：保持目標高度，前饋歸零

print(f"\n[Step 3c] Waiting for altitude (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s, "
      f"timeout {ALT_WAIT_TIMEOUT:.0f} s)...")
stable_since = None
t_ph = time.time()
while True:
    now = time.time()
    stable_since = (stable_since or now) if abs(TARGET_ALT_M + _ned['z']) < ALT_TOL else None
    if stable_since and now - stable_since >= HOVER_STABLE_TIME:
        print(f"  Stable at {-_ned['z']:.2f} m.")
        break
    if now - t_ph > ALT_WAIT_TIMEOUT:
        print(f"  Timeout – alt={-_ned['z']:.2f} m, continuing.")
        break
    time.sleep(0.05)

print(f"\n[Step 4] Level hover for {HOVER_BEFORE_S:.0f} s...")
time.sleep(HOVER_BEFORE_S)

# ─────────────────────────────────────────────────────────────
# Step 5 – roll + pitch 正弦追蹤（PARAM_SET @ PARAM_HZ，不等 ACK）
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 5] Roll/pitch sine tracking for {_TOTAL_S:.0f} s")
print(f"  {'t':>5}  {'env':>4}  {'roll_cmd':>8} {'fcu':>6} {'roll':>6}  "
      f"{'pitch_cmd':>9} {'fcu':>6} {'pitch':>6}  {'alt':>5}  {'xy_err':>6}  {'servo':>5}")

last_sent  = {'PFA_DES_ROLL': 0.0, 'PFA_DES_PITCH': 0.0}
n_sent     = 0
err_r, err_p, err_xy = [], [], []
tilt_cmd, tilt_meas = [], []   # 合成傾角 (deg)
aborted    = False
param_dt   = 1.0 / PARAM_HZ
elapsed    = 0.0
with _param_cv:
    for k in _param_acks:
        _param_acks[k] = 0

try:
    t0         = time.time()
    last_print = 0.0
    next_tick  = 0.0
    while True:
        t = time.time() - t0
        if t >= _TOTAL_S:
            break
        if not flight_ok():
            mode = _PX4_MODE_NAMES.get(_hb['main_mode'], _hb['main_mode'])
            print(f"\n  ✗ Left OFFBOARD / disarmed (mode={mode}, armed={_hb['armed']}) "
                  f"→ aborting sine")
            aborted = True
            break

        # 以實際時間計算指令；不等 ACK，掉包時下一拍會送新值
        roll_c, pitch_c = attitude_cmd_deg(t)
        for name, val in (('PFA_DES_ROLL', roll_c), ('PFA_DES_PITCH', pitch_c)):
            val = round(val, 2)
            if abs(val - last_sent[name]) >= PARAM_DEADBAND_DEG:
                param_set_nowait(name, val)
                last_sent[name] = val
                n_sent += 1

        # 追蹤誤差：與 FCU 回報的參數值比較（ATTITUDE 為 NED/FRD，與 PX4 同號）
        fcu_r   = fcu_param('PFA_DES_ROLL', 0.0)
        fcu_p   = fcu_param('PFA_DES_PITCH', 0.0)
        roll_m  = math.degrees(_att['roll'])
        pitch_m = math.degrees(_att['pitch'])
        xy_e    = math.hypot(hover_p[0] - _ned['x'], hover_p[1] - _ned['y'])
        if ENV_RAMP_S <= t < ENV_RAMP_S + _FULL_S:
            err_r.append(fcu_r - roll_m)
            err_p.append(fcu_p - pitch_m)
            err_xy.append(xy_e)
            tilt_cmd.append(math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(fcu_r)) * math.cos(math.radians(fcu_p)))))))
            tilt_meas.append(math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(roll_m)) * math.cos(math.radians(pitch_m)))))))

        now = time.time()
        if now - last_print >= LOG_PERIOD_S:
            sv = servo_max_deg()
            sv_s = f"{sv:4.1f}°" if sv is not None else "  n/a"
            print(f"  {t:5.1f}  {envelope(t):4.2f}  {roll_c:+8.1f} {fcu_r:+6.1f} {roll_m:+6.1f}  "
                  f"{pitch_c:+9.1f} {fcu_p:+6.1f} {pitch_m:+6.1f}  {-_ned['z']:5.2f}  "
                  f"{xy_e:6.2f}  {sv_s}")
            last_print = now

        next_tick += param_dt
        wait = next_tick - (time.time() - t0)
        if wait > 0:
            time.sleep(wait)
        else:
            next_tick = time.time() - t0   # 落後就不補送，從現在重新排程
    elapsed = time.time() - t0
finally:
    print("  PFA_DES_ROLL/PITCH → 0°")
    zero_attitude_params()

with _param_cv:
    acks = dict(_param_acks)
print(f"\n  PARAM_SET: {n_sent} sent in {elapsed:.1f} s "
      f"(≈ {n_sent / max(elapsed, 1e-3) / 2:.1f} Hz per axis), "
      f"PARAM_VALUE received: roll {acks['PFA_DES_ROLL']}, pitch {acks['PFA_DES_PITCH']}")


def _rms(x):
    return math.sqrt(sum(v * v for v in x) / len(x)) if x else float('nan')


if err_r:
    print(f"  Full-amplitude phase: roll err RMS {_rms(err_r):.1f}° (max {max(map(abs, err_r)):.1f}°)  "
          f"pitch err RMS {_rms(err_p):.1f}° (max {max(map(abs, err_p)):.1f}°)  "
          f"xy err RMS {_rms(err_xy):.2f} m (max {max(err_xy):.2f} m)")
    print(f"  Total tilt: cmd {min(tilt_cmd):.1f}–{max(tilt_cmd):.1f}°  "
          f"measured mean {sum(tilt_meas) / len(tilt_meas):.1f}° "
          f"(range {min(tilt_meas):.1f}–{max(tilt_meas):.1f}°)")

if aborted:
    # 已被 RC / failsafe 接手：不再切模式，避免搶走飛手的控制
    print("  Aborted – 不切換模式，交由目前模式 / 飛手處理。")
    shutdown(1)

# ─────────────────────────────────────────────────────────────
# Step 6 – 水平懸停 → 降落
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 6] Level hover for {HOVER_AFTER_S:.0f} s...")
time.sleep(HOVER_AFTER_S)

print("\n[Land] Landing (AUTO.LAND)...")
set_mode_land()
# 確認已離開 OFFBOARD 才停止 setpoint，否則會先觸發 offboard lost → Position mode
if not wait_for_mode(4, 5.0):
    print("  WARNING: AUTO.LAND not confirmed, retrying...")
    set_mode_land()
    wait_for_mode(4, 5.0)
_sp_active.clear()   # 停止 setpoint 串流
print("  讓 AUTO.LAND 自行偵測落地後上鎖，請勿在空中切 RC 上鎖開關。")
time.sleep(15)
_running.clear()
print("Done.")
