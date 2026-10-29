#!/usr/bin/env python3
"""
Circle Trajectory with Feedforward (pymavlink)
===============================================
circle_traj_ff_smooth_mavros.py 的 pymavlink 版本：繞圓形軌跡飛行，roll = pitch = 0°，
yaw 跟隨切線方向，同時送出位置 + 速度 + 加速度前饋（含向心加速度）。

■ 與 MAVROS 版的差異
  - 不需要 ROS：直接用 pymavlink 送 SET_POSITION_TARGET_LOCAL_NED（NED 座標，
    type_mask 只忽略 yaw_rate）。MAVROS 版的 ENU 幾何在這裡直接換成 NED。
  - 不下載參數、不做 timesync、只要求 2 個遙測串流（5 Hz）→ 經 57600 數傳也很輕。
  - 兩個背景執行緒：_rx_worker 唯一讀 socket、_sp_worker 以 SP_RATE_HZ 送 setpoint
    並每秒送 heartbeat；主執行緒的任何等待都不會中斷串流。
  - 平滑起飛（同 *_ff_smooth_mavros.py）：解鎖前 setpoint = 地面位置與航向，
    等 PX4 起飛緩升結束後以最小 jerk 曲線爬升。
  - 所有 yaw 轉向都用最小 jerk 曲線（MAVROS 版是階躍），轉完再停 YAW_SETTLE_S 才出發。
  - 路徑中心 = 懸停 setpoint（地面起飛點正上方）。
  - 若途中離開 OFFBOARD 或被上鎖：立即停止，不再切模式，交給飛手 / failsafe。

■ 圓形軌跡（NED，以路徑中心 (cn, ce) 為圓心）
  起點 = 圓的東側 (cn, ce + R)；角度 φ 由東方起算，CCW（俯視逆時針）為正：
    N = cn + R·sin φ        E = ce + R·cos φ        φ = dir·ω·τ，dir = +1 CCW / −1 CW
  τ(t)：時間扭曲，前後各 RAMP_S 秒 smoothstep 加減速（幾何不變，起停速度、加速度為 0）
  前饋：v = dp/dτ·τ̇，a = d²p/dτ²·τ̇² + dp/dτ·τ̈（等速段向心加速度 = v²/R）
  yaw = atan2(dE/dτ, dN/dτ)（切線方向，NED：0 = 北、+90° = 東）

■ 連線（MAVProxy 分流時）
  mavproxy.py --master=/dev/ttyUSB0 --baud 57600 \\
              --out udpout:127.0.0.1:14550 --out udpin:0.0.0.0:14551
  預設連 udpout:127.0.0.1:14551（MAVROS 不能同時使用 14551）；也可用參數指定：
    python3 circle_traj_ff.py udp:127.0.0.1:14550    （QGC 不能同時佔 14550）
"""

import sys
import time
import math
import threading
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration（預設與 circle_traj_ff_smooth_mavros.py 相同）
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 0.8    # 飛行高度 (m)，光流室內建議 0.5~1.0 m
CIRCLE_DIAMETER_M = 1.0    # 圓直徑 (m)
SPEED_MPS         = 0.3    # 切線速度 (m/s)
RAMP_S            = 3.0    # 起始加速 / 結束減速時間 (s)
NUM_LAPS          = 2      # 完整圓圈數
CLOCKWISE         = False  # False: 逆時針 (CCW); True: 順時針 (CW)，由上往下看
YAW_TRACK_PATH    = True   # True: yaw 追蹤切線方向；False: 維持起飛時的航向
APPROACH_PEAK_MPS = 0.3    # 中心 ↔ 起點的峰值速度 (m/s)
YAW_SETTLE_S      = 1.5    # yaw 轉完後再停留的時間 (s)

# MAVLink 連線：可用第一個參數覆寫，例如
#   python3 circle_traj_ff.py udp:127.0.0.1:14550        （監聽 MAVProxy 的 udpout；QGC 不能同時佔 14550）
# 預設接 MAVProxy 的 `--out udpin:0.0.0.0:14551`（MAVROS 不能同時使用 14551）
CONNECTION        = sys.argv[1] if len(sys.argv) > 1 else 'udpout:127.0.0.1:14551'

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

_R         = CIRCLE_DIAMETER_M / 2.0                       # 半徑 (m)
_OMEGA     = SPEED_MPS / _R                                # 角速度 (rad/s)
_DIR       = -1.0 if CLOCKWISE else 1.0
_PATH_S    = NUM_LAPS * 2.0 * math.pi / _OMEGA             # 等速時繞完的時間 (s)
_TOTAL_S   = _PATH_S + RAMP_S                              # 含加減速的總時間 (s)
_LAP_DUR_S = 2.0 * math.pi / _OMEGA                        # 等速時每圈時間 (s)
_ACC_C     = SPEED_MPS ** 2 / _R                           # 向心加速度 (m/s²)

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
# Trajectory building blocks（fn(t) -> (p, v, a, yaw)，NED）
# ─────────────────────────────────────────────────────────────

def line_duration(p0, p1, peak_speed, min_dur=0.0):
    """直線段時間：讓最小 jerk 曲線的峰值速度 = peak_speed（至少 min_dur）。"""
    dist = math.dist(p0, p1)
    return max(min_dur, MIN_JERK_PEAK * dist / peak_speed) if dist > 1e-6 else min_dur


def line_segment(p0, p1, T, yaw):
    """p0 → p1 的直線段（最小 jerk），yaw 固定。"""
    d = tuple(b - a for a, b in zip(p0, p1))

    def fn(t):
        s, sd, sdd = min_jerk(t, T)
        return (tuple(a + s * k for a, k in zip(p0, d)),
                tuple(sd * k for k in d),
                tuple(sdd * k for k in d),
                yaw)
    return fn


def ramp_time(t, path_dur, ramp):
    """時間扭曲 τ(t)：τ̇ 在前後 ramp 秒內以 smoothstep 3u²−2u³ 平滑 0↔1，中段 τ̇ = 1。

    回傳 (τ, τ̇, τ̈)；τ ∈ [0, path_dur]，總時間 = path_dur + ramp。
    """
    total = path_dur + ramp
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= total:
        return path_dur, 0.0, 0.0
    if t < ramp:
        u = t / ramp
        return ramp * u**3 * (1.0 - 0.5 * u), u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u) / ramp
    if t > total - ramp:
        r = (total - t) / ramp
        return path_dur - ramp * r**3 * (1.0 - 0.5 * r), r * r * (3.0 - 2.0 * r), -6.0 * r * (1.0 - r) / ramp
    return t - 0.5 * ramp, 1.0, 0.0


def circle_state(cn, ce, t, yaw_fixed):
    """t 秒時的 (p, v, a, yaw)，NED，以 (cn, ce) 為圓心、從東側起點出發。"""
    tau, td, tdd = ramp_time(t, _PATH_S, RAMP_S)
    w = _DIR * _OMEGA
    phi = w * tau
    s, c = math.sin(phi), math.cos(phi)
    p   = (cn + _R * s, ce + _R * c, -TARGET_ALT_M)
    dp  = (_R * c * w, -_R * s * w)                 # dp/dτ
    ddp = (-_R * s * w * w, -_R * c * w * w)        # d²p/dτ²
    v = (dp[0] * td, dp[1] * td, 0.0)
    a = (ddp[0] * td * td + dp[0] * tdd, ddp[1] * td * td + dp[1] * tdd, 0.0)
    # 切線方向只取決於 dp/dτ（τ̇ ≥ 0），起停時 yaw 不跳
    yaw = math.atan2(dp[1], dp[0]) if YAW_TRACK_PATH else yaw_fixed
    return p, v, a, yaw


print(f"  Circle (feedforward: p + v + a): D={CIRCLE_DIAMETER_M:.2f} m, {SPEED_MPS:.2f} m/s, "
      f"{'CW' if CLOCKWISE else 'CCW'}, {NUM_LAPS} laps, {_LAP_DUR_S:.1f} s/lap, {_TOTAL_S:.0f} s total")
print(f"    centripetal accel {_ACC_C:.2f} m/s², yaw rate {math.degrees(_OMEGA):.0f}°/s")
warn_speed(SPEED_MPS, "circle")

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
    # 路徑中心 = 懸停 setpoint（不是當下估測位置，避免把懸停誤差帶進路徑）
    cn, ce = hover_p[0], hover_p[1]
    print(f"  Path centre: NED ({cn:.2f}, {ce:.2f}) m")

    centre = (cn, ce, -TARGET_ALT_M)
    p0, _, _, yaw0 = circle_state(cn, ce, 0.0, ground_yaw)

    # ─────────────────────────────────────────────────────────
    # Step 4 – 中心平滑轉向起始切線方向 → 移動到起點（東側）
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 4] Approach start point N {p0[0]:.2f} E {p0[1]:.2f}, yaw → {math.degrees(yaw0):.0f}°")
    turn_and_hold(centre, ground_yaw, yaw0, yaw_turn_duration(ground_yaw, yaw0),
                  f"centre: yaw {math.degrees(ground_yaw):.0f}° → {math.degrees(yaw0):.0f}°")
    turn_and_hold(centre, yaw0, yaw0, YAW_SETTLE_S, "centre: yaw settle")
    T_app = line_duration(centre, p0, APPROACH_PEAK_MPS, min_dur=3.0)
    run_trajectory(line_segment(centre, p0, T_app, yaw0), T_app, "centre → start")
    turn_and_hold(p0, yaw0, yaw0, YAW_SETTLE_S, "start point settle")

    # ─────────────────────────────────────────────────────────
    # Step 5 – 圓形軌跡
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 5] Circle trajectory ({NUM_LAPS} lap(s), "
          f"{'CW' if CLOCKWISE else 'CCW'})  ~{_TOTAL_S:.0f} s total")
    p_end, _, _, yaw_end = run_trajectory(lambda t: circle_state(cn, ce, t, ground_yaw),
                                          _TOTAL_S, "circle")
    print(f"\n  Circle trajectory complete ({NUM_LAPS} lap(s)).")

    # ─────────────────────────────────────────────────────────
    # Step 6 – 回到中心，平滑轉回起飛航向並保持
    # ─────────────────────────────────────────────────────────
    print("\n[Step 6] Returning to circle centre...")
    T_ret = line_duration(p_end, centre, APPROACH_PEAK_MPS, min_dur=3.0)
    run_trajectory(line_segment(p_end, centre, T_ret, yaw_end), T_ret, "end → centre")
    turn_and_hold(centre, yaw_end, ground_yaw, max(5.0, yaw_turn_duration(yaw_end, ground_yaw)),
                  "centre hold, yaw → takeoff heading")

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
