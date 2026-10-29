#!/usr/bin/env python3
"""
Square Trajectory with Feedforward (pymavlink)
===============================================
square_traj_ff_mavros.py 的 pymavlink 版本：繞方形軌跡飛行，roll = pitch = 0°，
yaw 朝向當前邊的前進方向，同時送出位置 + 速度 + 加速度前饋。

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

■ 方形軌跡（NED，以路徑中心 (cn, ce) 為中心，邊長 L，S = L/2；頂點順序與 MAVROS 版相同）
  CCW：SE → NE → NW → SW → SE      CW：SE → SW → NW → NE → SE（由上往下看）
  每邊為最小 jerk 曲線 s(u) = 10u³ − 15u⁴ + 6u⁵（頂點處速度、加速度為 0），
  峰值速度 = SEG_PEAK_SPEED_MPS，每邊時間 T = 1.875 × L / 峰值速度。
  頂點：先以最小 jerk 曲線轉 90°（峰值角速度 VERTEX_YAW_RATE_DPS），
        再停 YAW_SETTLE_S 讓 yaw 穩定，才出發下一邊。

■ 連線（MAVProxy 分流時）
  mavproxy.py --master=/dev/ttyUSB0 --baud 57600 \\
              --out udpout:127.0.0.1:14550 --out udpin:0.0.0.0:14551
  預設連 udpout:127.0.0.1:14551（MAVROS 不能同時使用 14551）；也可用參數指定：
    python3 square_traj_ff.py udp:127.0.0.1:14550    （QGC 不能同時佔 14550）
"""

import sys
import time
import math
import threading
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration（幾何 / 速度預設與 square_traj_ff_mavros.py 相同）
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M        = 0.8    # 飛行高度 (m)，光流室內建議 0.5~1.0 m
SQUARE_SIDE_M       = 1.0    # 方形邊長 (m)
SEG_PEAK_SPEED_MPS  = 0.3    # 每邊最小 jerk 曲線的峰值速度 (m/s)
NUM_LAPS            = 2      # 完整方形圈數
CLOCKWISE           = False  # False: 逆時針 (CCW); True: 順時針 (CW)，由上往下看
YAW_TRACK_PATH      = True   # True: yaw 追蹤前進方向；False: 維持起飛時的航向
VERTEX_YAW_RATE_DPS = 45.0   # 頂點轉向的峰值角速度 (°/s)；90° 約 3.75 s
YAW_SETTLE_S        = 1.5    # 轉向完成後再停留的時間 (s)
APPROACH_MIN_S      = 3.0    # 中心 ↔ 第一個頂點的最短移動時間 (s)

# MAVLink 連線：可用第一個參數覆寫，例如
#   python3 square_traj_ff.py udp:127.0.0.1:14550        （監聽 MAVProxy 的 udpout；QGC 不能同時佔 14550）
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

_SEG_DUR_S = MIN_JERK_PEAK * SQUARE_SIDE_M / SEG_PEAK_SPEED_MPS     # 每邊飛行時間
_TURN_S    = (max(YAW_MIN_TURN_S, MIN_JERK_PEAK * (math.pi / 2) / math.radians(VERTEX_YAW_RATE_DPS))
              if YAW_TRACK_PATH else 0.0)                           # 頂點 90° 轉向時間
_LAP_DUR_S = 4 * (_SEG_DUR_S + _TURN_S + YAW_SETTLE_S)              # 每圈總時間

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


def vertex_turn(p, yaw0, yaw1, title):
    """頂點：最小 jerk 轉向（峰值 VERTEX_YAW_RATE_DPS）→ 停 YAW_SETTLE_S。"""
    d = abs(wrap_pi(yaw1 - yaw0))
    if d > 1e-3:
        T = max(YAW_MIN_TURN_S, MIN_JERK_PEAK * d / math.radians(VERTEX_YAW_RATE_DPS))
        turn_and_hold(p, yaw0, yaw1, T, title)
    turn_and_hold(p, yaw1, yaw1, YAW_SETTLE_S, title + " settle")


print(f"  Square (feedforward: p + v + a): side {SQUARE_SIDE_M:.2f} m, peak {SEG_PEAK_SPEED_MPS:.2f} m/s "
      f"→ {_SEG_DUR_S:.1f} s/side, {'CW' if CLOCKWISE else 'CCW'}")
print(f"    Vertex: turn {_TURN_S:.1f} s + settle {YAW_SETTLE_S:.1f} s, "
      f"{NUM_LAPS} laps ≈ {NUM_LAPS * _LAP_DUR_S:.0f} s")
print(f"    Peak acceleration ≈ {5.7735 * SQUARE_SIDE_M / _SEG_DUR_S ** 2:.2f} m/s²")
warn_speed(SEG_PEAK_SPEED_MPS, "square")

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

    S = SQUARE_SIDE_M / 2.0
    # NED (N, E)：SE = (cn − S, ce + S)
    if CLOCKWISE:
        verts  = [(cn - S, ce + S), (cn - S, ce - S), (cn + S, ce - S), (cn + S, ce + S)]
        labels = ['SE', 'SW', 'NW', 'NE']
    else:
        verts  = [(cn - S, ce + S), (cn + S, ce + S), (cn + S, ce - S), (cn - S, ce - S)]
        labels = ['SE', 'NE', 'NW', 'SW']
    for k, (vn, ve) in enumerate(verts):
        print(f"    v[{k}] {labels[k]}: N {vn:6.2f}  E {ve:6.2f} m")

    def vert3(i):
        return (verts[i][0], verts[i][1], -TARGET_ALT_M)

    def seg_yaw(src_i, dst_i):
        if not YAW_TRACK_PATH:
            return ground_yaw
        return math.atan2(verts[dst_i][1] - verts[src_i][1], verts[dst_i][0] - verts[src_i][0])

    centre   = (cn, ce, -TARGET_ALT_M)
    init_yaw = seg_yaw(0, 1)

    # ─────────────────────────────────────────────────────────
    # Step 4 – 中心平滑轉向第一邊方向 → 前往 v[0]
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 4] Turning to the first side heading, approaching v[0] ({labels[0]})...")
    turn_and_hold(centre, ground_yaw, init_yaw, yaw_turn_duration(ground_yaw, init_yaw),
                  f"centre: yaw {math.degrees(ground_yaw):.0f}° → {math.degrees(init_yaw):.0f}°")
    turn_and_hold(centre, init_yaw, init_yaw, YAW_SETTLE_S, "centre: yaw settle")
    T_app = line_duration(centre, vert3(0), SEG_PEAK_SPEED_MPS, min_dur=APPROACH_MIN_S)
    run_trajectory(line_segment(centre, vert3(0), T_app, init_yaw), T_app, f"centre → {labels[0]}")
    turn_and_hold(vert3(0), init_yaw, init_yaw, YAW_SETTLE_S, f"{labels[0]} settle")

    # ─────────────────────────────────────────────────────────
    # Step 5 – 方形軌跡
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 5] Square trajectory: {NUM_LAPS} lap(s) × 4 sides ≈ {NUM_LAPS * _LAP_DUR_S:.0f} s total")
    yaw = init_yaw
    for lap in range(NUM_LAPS):
        print(f"\n  ══ Lap {lap + 1}/{NUM_LAPS} ══")
        for seg in range(4):
            src_i, dst_i = seg, (seg + 1) % 4
            yaw      = seg_yaw(src_i, dst_i)
            next_yaw = seg_yaw(dst_i, (dst_i + 1) % 4)
            run_trajectory(line_segment(vert3(src_i), vert3(dst_i), _SEG_DUR_S, yaw), _SEG_DUR_S,
                           f"Side {seg + 1}/4  {labels[src_i]} → {labels[dst_i]}  yaw={math.degrees(yaw):.0f}°")
            last = (lap == NUM_LAPS - 1 and seg == 3)
            if last:
                next_yaw = yaw            # 最後一個頂點不轉，直接回中心
            vertex_turn(vert3(dst_i), yaw, next_yaw,
                        f"{labels[dst_i]}: yaw → {math.degrees(next_yaw):.0f}°")
            yaw = next_yaw
    print(f"\n  Square trajectory complete ({NUM_LAPS} lap(s)).")

    # ─────────────────────────────────────────────────────────
    # Step 6 – 回到中心，平滑轉回起飛航向並保持
    # ─────────────────────────────────────────────────────────
    print("\n[Step 6] Returning to square centre...")
    T_ret = line_duration(vert3(0), centre, SEG_PEAK_SPEED_MPS, min_dur=APPROACH_MIN_S)
    run_trajectory(line_segment(vert3(0), centre, T_ret, yaw), T_ret, f"{labels[0]} → centre")
    turn_and_hold(centre, yaw, ground_yaw, max(5.0, yaw_turn_duration(yaw, ground_yaw)),
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
