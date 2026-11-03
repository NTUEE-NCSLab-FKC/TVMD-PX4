#!/usr/bin/env python3
"""
PFA Sin Wave Path Tracking with Feedforward (pymavlink)
========================================================
pfa_sin_path.py 的前饋版：沿 PATH_HEADING_DEG 方向前進，側向以正弦波擺動，
roll = pitch = 0°，送出 位置 + 速度 + 加速度 前饋。機頭可設定：
  YAW_TRACK_PATH = True  → 機頭跟著軌跡切線方向（左右轉向）
  YAW_TRACK_PATH = False → 機頭固定朝前進方向（PATH_HEADING_DEG，PFA 平移不轉頭）

■ 與 pfa_sin_path.py 的差異
  - 前饋：舊版只送位置（type_mask 忽略速度 / 加速度），位置迴路要靠誤差才推得動，
    正弦的側向會明顯落後、振幅變小。本版送解析的 v、a。
  - 背景執行緒固定 SP_RATE_HZ 送 setpoint、另一執行緒專門讀 socket：舊版
    param_set() 等 ACK 時不送 setpoint，可能 offboard lost → RTL 飛回原點。
  - 平滑起飛（最小 jerk 爬升）、前後 RAMP_S 平滑加減速（舊版一開始側向速度就跳到 A·ω）。
  - 舊版預設 A = 1 m、T = 4 s：峰值水平速度 1.65 m/s 超過 pfa_pos_control 的
    _speed_xy_max（1 m/s）、側向加速度 2.5 m/s²，追不上；本版預設 T = 8 s（0.93 m/s、
    0.62 m/s²），啟動時檢查並警告。

■ 軌跡（以懸停點為原點，沿前進方向 u、側向 w = u 右轉 90°）
  幾何上是空間正弦：前進距離 = (λ/2π)·θ，側向 = A·sin θ，λ = PATH_SPEED_MPS × SIN_PERIOD_S。
  相位 θ(t) 前後各 RAMP_S 秒 smoothstep 加減速，中段 θ̇ = ω = 2π / SIN_PERIOD_S；
  前進與側向用同一個 θ，加減速時路徑形狀不變（不會在起點把正弦壓扁）。
    p  = c + (λ/2π)·θ·u + A·sin θ·w
    v  = [(λ/2π)·u + A·cos θ·w]·θ̇
    a  = −A·sin θ·θ̇²·w + [(λ/2π)·u + A·cos θ·w]·θ̈
    yaw（跟軌跡）= heading + atan2(A·cos θ, λ/2π)   ← 只取決於 dp/dθ，起停時不跳
  等速段峰值水平速度 √(V² + (A·ω)²)、峰值側向加速度 A·ω²；
  跟軌跡時峰值 yaw 角速度 = k·ω（θ = π/2 附近），k = 2πA/λ（最大轉向角 atan k）。

■ 連線（MAVProxy 分流時）
  mavproxy.py --master=/dev/ttyUSB0 --baud 57600 \
              --out udpout:127.0.0.1:14550 --out udpin:0.0.0.0:14551
  預設 udp:127.0.0.1:14550（與 pfa_sin_path.py 相同，QGC 不能同時佔 14550）；可用參數指定：
    python3 pfa_sin_path_ff.py udpout:127.0.0.1:14551   （實機經 MAVProxy）
    python3 pfa_sin_path_ff.py udpin:0.0.0.0:14540      （SITL）
"""

import sys
import time
import math
import threading
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M     = 1.0    # 飛行高度 (m)
PATH_SPEED_MPS   = 0.5    # 沿前進方向的平均速度 V (m/s)
PATH_HEADING_DEG = 0.0    # 前進方向 (deg，NED：0 = 北、90 = 東)；None = 起飛時的航向
SIN_AMPLITUDE_M  = 1.0    # 側向正弦振幅 A (m)，正值 = 先往前進方向的右側
SIN_PERIOD_S     = 8.0    # 振盪週期 T (s，等速段)；舊版 4 s 超過速度上限
NUM_CYCLES       = 4      # 完整週期數
RAMP_S           = 3.0    # 起始加速 / 結束減速時間 (s)
YAW_TRACK_PATH   = True   # True: 機頭跟軌跡切線方向；False: 機頭固定朝前進方向
YAW_RATE_WARN_DPS = 90.0  # 跟軌跡時峰值 yaw 角速度超過此值就警告

# MAVLink 連線：可用第一個參數覆寫，例如
#   python3 pfa_sin_path_ff.py udpout:127.0.0.1:14551  （實機：接 MAVProxy 的 --out udpin:0.0.0.0:14551）
#   python3 pfa_sin_path_ff.py udpin:0.0.0.0:14540     （SITL）
# 預設 udp:127.0.0.1:14550（與 pfa_sin_path.py 相同；QGC 不能同時佔 14550）
CONNECTION        = sys.argv[1] if len(sys.argv) > 1 else 'udp:127.0.0.1:14550'

HOVER_PHASE_DUR   = 8.0    # 起飛後定高懸停時間 (s)
ALT_TOL           = 0.15   # 穩定懸停高度容差 (m)
HOVER_STABLE_TIME = 3.0    # 穩定判定時間 (s)
ALT_WAIT_TIMEOUT  = 25.0   # 等待高度穩定的上限 (s)

# 平滑起飛（與 *_ff_smooth_mavros.py 相同）
CLIMB_PEAK_MPS    = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S       = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S  = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)
YAW_PEAK_RATE_DPS = 45.0   # 起飛後轉向起始航向的峰值角速度 (deg/s)
YAW_MIN_TURN_S    = 1.0    # 轉向最短時間 (s)

SP_RATE_HZ        = 10.0   # offboard setpoint 串流頻率；經 57600 數傳 10 Hz ≈ 650 B/s
LOG_PERIOD_S      = 0.5    # 追蹤狀態印出間隔 (s)
PX4_SPEED_XY_MAX  = 1.0    # pfa_pos_control 的 _speed_xy_max (m/s)，水平速度參考上限
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × L / T

# 遙測串流 (msg id → Hz)：LOCAL_POSITION_NED 5 Hz、ATTITUDE 5 Hz（數傳頻寬有限，維持低頻）
_STREAMS_HZ = {32: 5.0, 30: 5.0}

OMEGA        = 2.0 * math.pi / SIN_PERIOD_S
LAMBDA_M     = PATH_SPEED_MPS * SIN_PERIOD_S            # 空間波長 (m)
K_FWD        = LAMBDA_M / (2.0 * math.pi)               # 前進距離 / 相位 (m/rad)
THETA_END    = NUM_CYCLES * 2.0 * math.pi
TOTAL_DUR_S  = THETA_END / OMEGA + RAMP_S               # 含加減速的總時間
PATH_LEN_M   = K_FWD * THETA_END
V_PEAK_MPS   = OMEGA * math.hypot(K_FWD, SIN_AMPLITUDE_M)
A_LAT_MPS2   = abs(SIN_AMPLITUDE_M) * OMEGA ** 2
K_SLOPE      = SIN_AMPLITUDE_M / K_FWD                  # 側向 / 前進 斜率峰值
YAW_MAX_DEG  = math.degrees(math.atan(abs(K_SLOPE)))    # 跟軌跡時最大轉向角
YAW_RATE_PEAK_DPS = math.degrees(abs(K_SLOPE) * OMEGA)

# ─────────────────────────────────────────────────────────────
# Connect
# udpout 連線必須先送出封包，MAVProxy 的 udpin 才知道要回傳給誰，
# 所以先送 heartbeat（MAV_TYPE_ONBOARD_CONTROLLER），之後由 _sp_worker 每秒補送。
# 只接受飛控本身的 heartbeat（排除 MAVProxy / QGC 的 GCS heartbeat）。
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
        sys.exit("  ERROR: 30 s 內沒有收到飛控 heartbeat（檢查 MAVProxy 輸出埠 / 連線字串）")
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")

for _msg_id, _hz in _STREAMS_HZ.items():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(_msg_id), 1e6 / _hz, 0, 0, 0, 0, 0)
    time.sleep(0.05)


# ─────────────────────────────────────────────────────────────
# Telemetry state cache（只由 _rx_worker 寫入，主執行緒只讀）
# ─────────────────────────────────────────────────────────────
_ned    = {'x': 0.0, 'y': 0.0, 'z': -0.01, 't': 0.0}            # LOCAL_POSITION_NED
_att    = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 't': 0.0}     # ATTITUDE (NED/FRD)
_hb     = {'main_mode': None, 'armed': False, 't': 0.0}          # 飛控 HEARTBEAT
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
# PX4 offboard_control_mode 超過 COM_OF_LOSS_T 未更新 → offboard signal lost。
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


def param_set(name: str, value: float, retries: int = 5, timeout: float = 1.5) -> bool:
    """PARAM_SET + 等待本次送出之後的 PARAM_VALUE，並確認數值一致。"""
    nb = name.encode('utf-8')
    for attempt in range(retries):
        t_sent = time.time()
        _send(master.mav.param_set_send, master.target_system, master.target_component,
              nb, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        with _param_cv:
            got = _param_cv.wait_for(
                lambda: name in _params and _params[name][1] >= t_sent, timeout)
            actual = _params[name][0] if got else None
        if got and abs(actual - value) < 0.01:
            return True
    return False


def param_get(name: str, timeout: float = 3.0):
    t_sent = time.time()
    _send(master.mav.param_request_read_send, master.target_system, master.target_component,
          name.encode('utf-8'), -1)
    with _param_cv:
        if _param_cv.wait_for(lambda: name in _params and _params[name][1] >= t_sent, timeout):
            return _params[name][0]
    return None


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


def run_trajectory(fn, duration, title):
    """以 fn(t) -> (p, v, a, yaw)（NED）送出 setpoint 直到 duration，定期印出追蹤誤差。"""
    print(f"  ── {title}  ({duration:.1f} s) ──")
    t0 = time.time()
    last_log = 0.0
    while True:
        check_flight()
        t = min(time.time() - t0, duration)
        p, v, a, yaw = fn(t)
        set_pva(p, v, a, yaw)
        now = time.time()
        if now - last_log >= LOG_PERIOD_S:
            xn, yn, zn = _ned['x'], _ned['y'], _ned['z']
            print(f"    {t:5.1f}s  cmd N/E=({p[0]:6.2f},{p[1]:6.2f}) alt={-p[2]:4.2f}  "
                  f"pos=({xn:6.2f},{yn:6.2f}) alt={-zn:4.2f}  "
                  f"err={math.hypot(p[0] - xn, p[1] - yn):4.2f}m  "
                  f"|v_ff|={math.hypot(v[0], v[1]):4.2f}  |a_ff|={math.hypot(a[0], a[1]):4.2f}  "
                  f"yaw={math.degrees(yaw):6.1f}° (now {math.degrees(_att['yaw']):6.1f}°)")
            last_log = now
        if t >= duration:
            return fn(duration)
        time.sleep(0.05)


def turn_and_hold(p, yaw0, yaw1, duration, title):
    """在 p 保持（零速度 / 加速度前饋），yaw 以最小 jerk 曲線從 yaw0 轉到 yaw1。"""
    d = wrap_pi(yaw1 - yaw0)

    def fn(t):
        s, _, _ = min_jerk(t, duration)
        return p, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), wrap_pi(yaw0 + s * d)
    run_trajectory(fn, duration, title)


def warn_speed(peak_xy_mps, what):
    if peak_xy_mps > PX4_SPEED_XY_MAX:
        print(f"  WARNING: {what} peak horizontal speed {peak_xy_mps:.2f} m/s > "
              f"pfa_pos_control _speed_xy_max {PX4_SPEED_XY_MAX:.1f} m/s "
              f"→ velocity feedforward will be clipped. Slow down or shrink the path.")


# ─────────────────────────────────────────────────────────────
# Figure-8 trajectory (analytic position / velocity / acceleration, NED)
# ─────────────────────────────────────────────────────────────

def sin_phase(t):
    """回傳 (θ, θ̇, θ̈)：前後 RAMP_S 秒以 smoothstep 平滑加減速，中段等角速度 ω。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= TOTAL_DUR_S:
        return THETA_END, 0.0, 0.0
    if t < RAMP_S:
        u = t / RAMP_S
        return (OMEGA * RAMP_S * u**3 * (1.0 - 0.5 * u),
                OMEGA * u * u * (3.0 - 2.0 * u),
                OMEGA * 6.0 * u * (1.0 - u) / RAMP_S)
    if t > TOTAL_DUR_S - RAMP_S:
        r = (TOTAL_DUR_S - t) / RAMP_S
        return (THETA_END - OMEGA * RAMP_S * r**3 * (1.0 - 0.5 * r),
                OMEGA * r * r * (3.0 - 2.0 * r),
                -OMEGA * 6.0 * r * (1.0 - r) / RAMP_S)
    return 0.5 * OMEGA * RAMP_S + OMEGA * (t - RAMP_S), OMEGA, 0.0


def sin_state(c, heading, t, yaw_fixed):
    """t 秒時的 (p, v, a, yaw)，NED。c = 起點 (N, E)，heading = 前進方向 (rad)。"""
    th, thd, thdd = sin_phase(t)
    uN, uE = math.cos(heading), math.sin(heading)          # 前進方向
    wN, wE = -math.sin(heading), math.cos(heading)         # 右側
    s1, c1 = math.sin(th), math.cos(th)
    A = SIN_AMPLITUDE_M
    f, l = K_FWD * th, A * s1                              # 前進距離、側向位移
    df, dl = K_FWD, A * c1                                 # 對 θ 的一階導數
    ddl = -A * s1                                          # 對 θ 的二階導數（前進為 0）
    vf, vl = df * thd, dl * thd
    af, al = df * thdd, ddl * thd * thd + dl * thdd
    p = (c[0] + f * uN + l * wN, c[1] + f * uE + l * wE, -TARGET_ALT_M)
    v = (vf * uN + vl * wN, vf * uE + vl * wE, 0.0)
    a = (af * uN + al * wN, af * uE + al * wE, 0.0)
    # 切線方向只取決於 dp/dθ（θ̇ ≥ 0），起停時 yaw 不跳
    yaw = wrap_pi(heading + math.atan2(dl, df)) if YAW_TRACK_PATH else yaw_fixed
    return p, v, a, yaw


print(f"  Sin path (feedforward: p + v + a): A=±{SIN_AMPLITUDE_M:.2f} m, T={SIN_PERIOD_S:.1f} s, "
      f"λ={LAMBDA_M:.2f} m, {NUM_CYCLES} cycles, {PATH_LEN_M:.1f} m, {TOTAL_DUR_S:.0f} s total")
print(f"    peak horizontal speed {V_PEAK_MPS:.2f} m/s, lateral accel {A_LAT_MPS2:.2f} m/s²")
print(f"    yaw: {'follows the path (max ±%.0f°, peak rate %.0f°/s)' % (YAW_MAX_DEG, YAW_RATE_PEAK_DPS) if YAW_TRACK_PATH else 'fixed to the path heading'}")
warn_speed(V_PEAK_MPS, "sin path")
if YAW_TRACK_PATH and YAW_RATE_PEAK_DPS > YAW_RATE_WARN_DPS:
    print(f"  WARNING: peak yaw rate {YAW_RATE_PEAK_DPS:.0f}°/s > {YAW_RATE_WARN_DPS:.0f}°/s → yaw 會跟不上；"
          f"加長 SIN_PERIOD_S、減小 SIN_AMPLITUDE_M 或設 YAW_TRACK_PATH = False")

# ─────────────────────────────────────────────────────────────
# Step 0 – PFA 姿態參數歸零、讀取起飛緩升參數
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
for _name in ('PFA_DES_ROLL', 'PFA_DES_PITCH'):
    print(f"  {_name} = 0 ... {'OK' if param_set(_name, 0.0) else 'FAILED (continuing)'}")

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
if tkf_byp is not None and int(round(tkf_byp)) == 0:
    print("  WARNING: PFA_TKF_BYP = 0 → OFFBOARD 下收不到 want_takeoff，推力上限會卡在 0.1，"
          "請改回 1。")
spool_s = 1.0 if spool_s is None else spool_s
ramp_s  = 3.0 if ramp_s  is None else ramp_s
TAKEOFF_WAIT_S = spool_s + ramp_s + TAKEOFF_SETTLE_S
print(f"  Takeoff ramp: COM_SPOOLUP_TIME={spool_s:.1f} s + MPC_TKO_RAMP_T={ramp_s:.1f} s "
      f"→ wait {TAKEOFF_WAIT_S:.1f} s after arming before climbing")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stream setpoints → OFFBOARD → Arm
# 解鎖前 setpoint = 目前地面位置與航向（高度誤差 0），而不是目標高度
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
        shutdown(1)
print("  Armed.")

try:
    # ─────────────────────────────────────────────────────────
    # Step 3 – 平滑起飛：等 PX4 起飛緩升 → 最小 jerk 爬升 → 等高度穩定
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 3a] Holding ground position for {TAKEOFF_WAIT_S:.1f} s (PX4 takeoff ramp)...")
    t_w = time.time()
    while time.time() - t_w < TAKEOFF_WAIT_S:
        check_flight()
        time.sleep(0.05)

    z0   = _ned['z']
    dz   = -TARGET_ALT_M - z0
    T_cl = max(CLIMB_MIN_S, MIN_JERK_PEAK * abs(dz) / CLIMB_PEAK_MPS)

    def _climb(t):
        s, sd, sdd = min_jerk(t, T_cl)
        return ((ground[0], ground[1], z0 + s * dz), (0.0, 0.0, sd * dz),
                (0.0, 0.0, sdd * dz), ground_yaw)
    print(f"\n[Step 3b] Smooth climb {-z0:.2f} m → {TARGET_ALT_M:.2f} m")
    run_trajectory(_climb, T_cl, "climb")
    hover_p = (ground[0], ground[1], -TARGET_ALT_M)
    set_pva(hover_p, yaw=ground_yaw)

    print(f"\n[Step 3c] Waiting for altitude (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")
    stable_since = None
    t_ph = last_log = time.time()
    while True:
        check_flight()
        now = time.time()
        alt_err = TARGET_ALT_M + _ned['z']
        stable_since = (stable_since or now) if abs(alt_err) < ALT_TOL else None
        if now - last_log > 1.0:
            print(f"  alt={-_ned['z']:.2f} m (err={-alt_err:+.2f})")
            last_log = now
        if stable_since and now - stable_since >= HOVER_STABLE_TIME:
            print(f"  Stable at {-_ned['z']:.2f} m.")
            break
        if now - t_ph > ALT_WAIT_TIMEOUT:
            print(f"  Timeout – alt={-_ned['z']:.2f} m, continuing.")
            break
        time.sleep(0.05)

    print(f"\n[Step 3d] Hover hold for {HOVER_PHASE_DUR:.0f} s...")
    t_w = time.time()
    while time.time() - t_w < HOVER_PHASE_DUR:
        check_flight()
        time.sleep(0.05)
    # 路徑起點 = 懸停 setpoint（不是當下估測位置，避免把懸停誤差帶進路徑）
    c0 = (hover_p[0], hover_p[1])
    heading = ground_yaw if PATH_HEADING_DEG is None else math.radians(PATH_HEADING_DEG)
    print(f"  Path start: NED ({c0[0]:.2f}, {c0[1]:.2f}) m, heading {math.degrees(heading):.0f}°")

    centre = (c0[0], c0[1], -TARGET_ALT_M)
    _, _, _, yaw0 = sin_state(c0, heading, 0.0, heading)

    # ─────────────────────────────────────────────────────────
    # Step 4 – 起點平滑轉向起始方向 → 正弦路徑
    # ─────────────────────────────────────────────────────────
    turn_and_hold(centre, ground_yaw, yaw0, yaw_turn_duration(ground_yaw, yaw0),
                  f"start: yaw {math.degrees(ground_yaw):.0f}° → {math.degrees(yaw0):.0f}°")

    print(f"\n[Step 4] Sin path (FF): {NUM_CYCLES} cycles, {TOTAL_DUR_S:.0f} s total")
    run_trajectory(lambda t: sin_state(c0, heading, t, heading), TOTAL_DUR_S, "sin path")
    print("\n  Sin path complete.")

    # ─────────────────────────────────────────────────────────
    # Step 5 – 終點保持，平滑轉回前進方向
    # ─────────────────────────────────────────────────────────
    p_end, _, _, yaw_end = sin_state(c0, heading, TOTAL_DUR_S, heading)
    print("\n[Step 5] Holding at the end point...")
    turn_and_hold(p_end, yaw_end, heading, max(5.0, yaw_turn_duration(yaw_end, heading)),
                  f"end hold, yaw → path heading {math.degrees(heading):.0f}°")

except Aborted as e:
    print(f"\n  ✗ Aborted: {e} → 不再切換模式，交給飛手 / failsafe。")
    shutdown(1)
except KeyboardInterrupt:
    print("\n  Ctrl-C → AUTO.LAND")

# ─────────────────────────────────────────────────────────────
# Land
# ─────────────────────────────────────────────────────────────
print("\n[Land] Landing (AUTO.LAND)...")
set_mode_land()
# 確認已離開 OFFBOARD 才停止 setpoint，否則會先觸發 offboard lost → Position mode
if not wait_for_mode(4, 5.0):
    print("  WARNING: AUTO.LAND not confirmed, retrying...")
    set_mode_land()
    wait_for_mode(4, 5.0)
_sp_active.clear()
print("  讓 AUTO.LAND 自行偵測落地後上鎖，請勿在空中切 RC 上鎖開關。")
time.sleep(15)
_running.clear()
print("Done.")
