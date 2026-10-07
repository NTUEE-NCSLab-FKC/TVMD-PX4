#!/usr/bin/env python3
"""
Square Trajectory with Feedforward (MAVROS / ROS 1)
=====================================================
光流室內版：繞 1 m × 1 m 方形軌跡飛行，同時送出位置 + 速度 + 加速度前饋。
square_traj_mavros.py（CCW）/ square_traj_cw_mavros.py（CW）的前饋版本，
以 CLOCKWISE 切換方向。

■ 為什麼要前饋
  只送位置時 trajectory_setpoint.velocity/acceleration = NaN → pfa_pos_control
  清成 0，D 項以 0 為速度參考一直煞車 → 固定落後 (Kv/Kp)·速度。
  送出 v_ref、a_ref 後誤差動態為 ë + Kv·ė + Kp·e = 0，落後收斂到 0。
  （前饋無法消除重心偏移等固定干擾造成的穩態誤差，那需要積分項。）

■ 軌跡（以懸停位置為中心，ENU）
  CCW：SE → NE → NW → SW → SE      CW：SE → SW → NW → NE → SE（由上往下看）
  每邊使用最小 jerk 曲線 s(t) = 10u³ − 15u⁴ + 6u⁵（u = t/T）：
    頂點處速度、加速度皆為 0 → 可以停下來轉 yaw，前饋連續
    峰值速度 = SEG_PEAK_SPEED_MPS，每邊時間 T = 1.875 × 邊長 / 峰值速度
  頂點停留 VERTEX_DWELL_S，yaw 轉向下一段方向

■ 平滑起飛（避免「往上衝 → 掉下來 → 再起飛」）
  若解鎖前就送 z = TARGET_ALT_M，pfa_pos_control 的起飛緩升（PFA_TKF_BYP=1）結束、
  切到正常位置控制的瞬間高度誤差 ≈ TARGET_ALT_M，P 項把推力推到接近滿載 → 往上衝；
  電流暴增造成外部電源壓降 → 推力不足又掉下來。因此：
    1. 解鎖前 setpoint = 目前地面位置與航向（高度誤差 0）
    2. 解鎖後維持地面位置，等 PX4 起飛緩升結束
       （COM_SPOOLUP_TIME + MPC_TKO_RAMP_T，從飛控讀取，再加 TAKEOFF_SETTLE_S）
    3. 以最小 jerk 曲線從地面高度爬升到 TARGET_ALT_M（峰值 CLIMB_PEAK_MPS），含前饋
  注意：OFFBOARD 下必須 PFA_TKF_BYP = 1，否則推力上限會卡在 0.1（啟動時會檢查）。

■ 控制鏈
  set_sp() → rospy.Timer (CTRL_HZ) → /mavros/setpoint_raw/local (PositionTarget)
  → SET_POSITION_TARGET_LOCAL_NED (p + v + a + yaw) → pfa_pos_control → pfa_att_control

執行方式：
  python3 square_traj_ff_mavros.py
"""

import sys
import math
import threading

import rospy
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ParamValue, PositionTarget
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
    ParamPull,   ParamPullRequest,
    ParamGet,    ParamGetRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M       = 0.8    # 懸停高度 (m)，光流室內建議 0.5~1.0 m
SQUARE_SIDE_M      = 1.0    # 方形邊長 (m)
SEG_PEAK_SPEED_MPS = 0.3    # 每邊最小 jerk 曲線的峰值速度 (m/s)
VERTEX_DWELL_S     = 1.5    # 各頂點停留時間 (s)，讓 yaw 完成轉向
NUM_LAPS           = 2      # 完整方形圈數
CLOCKWISE          = False  # False: 逆時針 (CCW); True: 順時針 (CW)，由上往下看
YAW_TRACK_PATH     = True   # True: yaw 追蹤前進方向; False: 固定 yaw=0°

HOVER_PHASE_DUR    = 5.0    # 起飛後穩定等待時間 (s)
ALT_TOL            = 0.10   # 高度穩定容忍 (m)
HOVER_STABLE_TIME  = 3.0    # 連續穩定秒數

# 平滑起飛
CLIMB_PEAK_MPS     = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S        = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S   = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)

CTRL_HZ           = 20     # Timer 發布頻率（經 SiK 數傳時建議降到 10）
LOG_PERIOD_S      = 0.5    # 追蹤狀態印出間隔 (s)
PX4_SPEED_XY_MAX  = 1.0    # pfa_pos_control 的 _speed_xy_max (m/s)，水平速度參考上限
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × L / T

# PositionTarget.type_mask：位置、速度、加速度、yaw 都使用；只忽略 yaw_rate
# （pfa_pos_control / pfa_att_control 目前不讀 trajectory_setpoint.yawspeed）
TYPE_MASK_PVA_YAW = PositionTarget.IGNORE_YAW_RATE

ZERO3 = (0.0, 0.0, 0.0)

# Derived
_SEG_DUR_S = MIN_JERK_PEAK * SQUARE_SIDE_M / SEG_PEAK_SPEED_MPS     # 每邊飛行時間
_LAP_DUR_S = 4 * (_SEG_DUR_S + VERTEX_DWELL_S)                      # 每圈總時間


# ─────────────────────────────────────────────────────────────
# Global state
# ─────────────────────────────────────────────────────────────
_vehicle_state = State()
_local_pose    = PoseStamped()
_imu_data      = Imu()


def _cb_state(msg): global _vehicle_state; _vehicle_state = msg
_pose_received = threading.Event()

def _cb_pose(msg):
    global _local_pose
    _local_pose = msg
    _pose_received.set()
def _cb_imu(msg):   global _imu_data;      _imu_data      = msg


def get_altitude():
    return _local_pose.pose.position.z

def get_xyz():
    p = _local_pose.pose.position
    return p.x, p.y, p.z

def get_yaw():
    o = _local_pose.pose.orientation
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

def get_roll_pitch_yaw_deg():
    o = _imu_data.orientation
    roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# ─────────────────────────────────────────────────────────────
# Setpoint helper
# ─────────────────────────────────────────────────────────────

def make_target(p, v=ZERO3, a=ZERO3, yaw_rad=0.0):
    """ENU PositionTarget：位置 + 速度 + 加速度 + yaw（全部為有限值）。

    三軸都必須是有限值：pfa_pos_control 的 _checkAllFinite() 只要任一分量
    為 NaN 就把整個向量清成 0。MAVROS 會把 ENU 轉成 NED 再送給 PX4。
    """
    sp = PositionTarget()
    sp.header.stamp     = rospy.Time.now()
    sp.header.frame_id  = 'map'
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    sp.type_mask        = TYPE_MASK_PVA_YAW
    sp.position.x, sp.position.y, sp.position.z = p
    sp.velocity.x, sp.velocity.y, sp.velocity.z = v
    sp.acceleration_or_force.x, sp.acceleration_or_force.y, sp.acceleration_or_force.z = a
    sp.yaw      = yaw_rad
    sp.yaw_rate = 0.0
    return sp


# ─────────────────────────────────────────────────────────────
# Shared setpoint（主執行緒寫入；Timer 背景讀取並發布）
# ─────────────────────────────────────────────────────────────
_sp_lock = threading.Lock()
_sp_msg  = [None]

def set_sp(msg):
    """執行緒安全地更新 setpoint（主執行緒呼叫）。"""
    with _sp_lock:
        _sp_msg[0] = msg

def _timer_cb(_event):
    """CTRL_HZ 背景回呼：更新 timestamp 後發布。"""
    with _sp_lock:
        msg = _sp_msg[0]
    if msg is not None:
        msg.header.stamp = rospy.Time.now()
        sp_pub.publish(msg)


# ─────────────────────────────────────────────────────────────
# Trajectory building blocks（每段回傳 fn(t) -> (p, v, a, yaw)，ENU）
# ─────────────────────────────────────────────────────────────

def min_jerk(t, T):
    """最小 jerk 曲線 s(t)∈[0,1]：回傳 (s, ṡ, s̈)，起訖速度與加速度皆為 0。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    u = t / T
    s   = u**3 * (10.0 - 15.0 * u + 6.0 * u * u)
    sd  = 30.0 * u * u * (1.0 - u) ** 2 / T
    sdd = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T)
    return s, sd, sdd


def line_duration(p0, p1, peak_speed, min_dur=0.0):
    """直線段時間：讓最小 jerk 曲線的峰值速度 = peak_speed（至少 min_dur）。"""
    dist = math.dist(p0, p1)
    return max(min_dur, MIN_JERK_PEAK * dist / peak_speed) if dist > 1e-6 else min_dur


def line_segment(p0, p1, T, yaw_rad):
    """p0 → p1 的直線段（最小 jerk），yaw 固定。"""
    d = tuple(b - a for a, b in zip(p0, p1))
    def fn(t):
        s, sd, sdd = min_jerk(t, T)
        return (tuple(a + s * k for a, k in zip(p0, d)),
                tuple(sd * k for k in d),
                tuple(sdd * k for k in d),
                yaw_rad)
    return fn


def run_trajectory(fn, duration, title):
    """以 fn(t) 送出 p/v/a/yaw 直到 duration，每 LOG_PERIOD_S 印出追蹤誤差。回傳終點狀態。"""
    rospy.loginfo(f"  ── {title}  ({duration:.1f} s) ──")
    t0       = rospy.Time.now()
    last_log = t0
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t   = min((now - t0).to_sec(), duration)
        p, v, a, yaw = fn(t)
        set_sp(make_target(p, v, a, yaw))

        if (now - last_log).to_sec() >= LOG_PERIOD_S:
            xn, yn, zn = get_xyz()
            rospy.loginfo(f"    {t:5.1f}s  cmd=({p[0]:6.2f},{p[1]:6.2f},{p[2]:5.2f})  "
                          f"pos=({xn:6.2f},{yn:6.2f},{zn:5.2f})  "
                          f"err={math.hypot(p[0] - xn, p[1] - yn):4.2f}m  "
                          f"|v_ff|={math.hypot(v[0], v[1]):4.2f}  |a_ff|={math.hypot(a[0], a[1]):4.2f}  "
                          f"yaw={math.degrees(yaw):6.1f}°")
            last_log = now
        if t >= duration:
            break
        rate.sleep()
    return fn(duration)


def hold(p, yaw_rad, duration, title):
    """在 p 以零速度/加速度前饋保持 duration 秒。"""
    rospy.loginfo(f"  ── {title}  ({duration:.1f} s) ──")
    set_sp(make_target(p, yaw_rad=yaw_rad))
    t_end    = rospy.Time.now() + rospy.Duration(duration)
    last_log = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < t_end:
        now = rospy.Time.now()
        if (now - last_log).to_sec() >= 2.0:
            xn, yn, zn = get_xyz()
            r_deg, p_deg, _ = get_roll_pitch_yaw_deg()
            rospy.loginfo(f"    pos=({xn:.2f},{yn:.2f}) alt={zn:.2f}m  "
                          f"err={math.hypot(p[0] - xn, p[1] - yn):.2f}m  "
                          f"roll={r_deg:+.1f}°  pitch={p_deg:+.1f}°")
            last_log = now
        rate.sleep()


def warn_speed(peak_xy_mps, what):
    if peak_xy_mps > PX4_SPEED_XY_MAX:
        rospy.logwarn(f"  WARNING: {what} peak horizontal speed {peak_xy_mps:.2f} m/s > "
                      f"pfa_pos_control _speed_xy_max {PX4_SPEED_XY_MAX:.1f} m/s "
                      f"→ velocity feedforward will be clipped. Slow down or shrink the path.")


# ─────────────────────────────────────────────────────────────
# MAVROS service wrappers
# ─────────────────────────────────────────────────────────────

def param_pull(force=True, timeout=15.0):
    """Sync MAVROS parameter cache from FCU (prevents stale-cache param_set failures)."""
    try:
        rospy.wait_for_service('/mavros/param/pull', timeout=timeout)
        svc = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
        res = svc(ParamPullRequest(force_pull=force))
        if res.success:
            rospy.loginfo(f"  param_pull: synced {res.param_received} parameters from FCU")
        else:
            rospy.logwarn("  param_pull: reported failure")
        return res.success
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"  param_pull failed: {e}")
        return False


def param_get(name):
    """讀取 FCU 參數（float 或 int），失敗回傳 None。"""
    try:
        rospy.wait_for_service('/mavros/param/get', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        res = svc(ParamGetRequest(param_id=name))
        if not res.success:
            return None
        return res.value.real if res.value.integer == 0 else float(res.value.integer)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"param_get({name}) failed: {e}")
        return None


def param_set(name, value, retries=5):
    try:
        rospy.wait_for_service('/mavros/param/set', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        for _ in range(retries):
            if svc(req).success:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"param_set({name}) failed: {e}")
    return False


def set_mode(mode_str, retries=5):
    try:
        rospy.wait_for_service('/mavros/set_mode', timeout=5)
        svc = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        for _ in range(retries):
            if svc(SetModeRequest(custom_mode=mode_str)).mode_sent:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"set_mode({mode_str}) failed: {e}")
    return False


def arm_vehicle(retries=5):
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        for _ in range(retries):
            if svc(CommandBoolRequest(value=True)).success:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"arming failed: {e}")
    return False


# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('square_traj_ff', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

sp_pub = rospy.Publisher(
    '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected. mode={_vehicle_state.mode}")

rospy.loginfo("Waiting for local position...")
while not rospy.is_shutdown() and not _pose_received.is_set():
    rate.sleep()

# 解鎖前 setpoint = 目前地面位置與航向（高度誤差 0），而不是 TARGET_ALT_M
ground     = get_xyz()
ground_yaw = get_yaw()
set_sp(make_target(ground, yaw_rad=ground_yaw))   # 初始 setpoint（Timer 啟動前先設定）
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_cb)
rospy.loginfo(f"  Ground position: ENU ({ground[0]:.2f}, {ground[1]:.2f}, {ground[2]:.2f}) m  "
              f"yaw={math.degrees(ground_yaw):.1f}°")
rospy.loginfo(f"  Plan: alt={TARGET_ALT_M}m  side={SQUARE_SIDE_M}m  "
              f"peak={SEG_PEAK_SPEED_MPS}m/s  {'CW' if CLOCKWISE else 'CCW'}  "
              f"{_SEG_DUR_S:.1f}s/side × 4 + {VERTEX_DWELL_S}s dwell × {NUM_LAPS} laps "
              f"≈ {_LAP_DUR_S * NUM_LAPS:.0f}s  (feedforward: p + v + a)")
warn_speed(SEG_PEAK_SPEED_MPS, "square")
rospy.loginfo("  Syncing parameter cache from FCU...")
param_pull()

# ─────────────────────────────────────────────────────────────
# Step 0 – PFA attitude parameters
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0°...")
rospy.loginfo("  PFA_DES_ROLL  = 0° ... " +
              ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH = 0° ... " +
              ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

tkf_byp = param_get('PFA_TKF_BYP')
spool_s = param_get('COM_SPOOLUP_TIME')
ramp_s  = param_get('MPC_TKO_RAMP_T')
if tkf_byp is not None and int(round(tkf_byp)) == 0:
    rospy.logwarn("  WARNING: PFA_TKF_BYP = 0 → OFFBOARD 下收不到 want_takeoff，"
                  "推力上限會卡在 0.1，請改回 1。")
spool_s = 1.0 if spool_s is None else spool_s
ramp_s  = 3.0 if ramp_s  is None else ramp_s
TAKEOFF_WAIT_S = spool_s + ramp_s + TAKEOFF_SETTLE_S
rospy.loginfo(f"  Takeoff ramp: COM_SPOOLUP_TIME={spool_s:.1f} s + MPC_TKO_RAMP_T={ramp_s:.1f} s "
              f"→ wait {TAKEOFF_WAIT_S:.1f} s after arming before climbing")

# ─────────────────────────────────────────────────────────────
# Step 1 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming ground-position setpoints (5 s)...")
# Timer 在背景發布，rospy.sleep() 讓主執行緒等待即可
rospy.sleep(5.0)

# arm_vehicle()/set_mode() 阻塞時，Timer 仍在背景以 CTRL_HZ 發布
rospy.loginfo("  Requesting OFFBOARD mode...")
set_mode('OFFBOARD')
t_wait = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t_wait).to_sec() > 5.0:
        rospy.logwarn("  WARNING: OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_vehicle_state.mode}")

rospy.loginfo("  Arming...")
arm_vehicle()
t_wait = rospy.Time.now()
while not rospy.is_shutdown() and not _vehicle_state.armed:
    if (rospy.Time.now() - t_wait).to_sec() > 10.0:
        rospy.logerr("  ERROR: Failed to arm.")
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 2 – 平滑起飛：等 PX4 起飛緩升結束 → 最小 jerk 爬升（含前饋）→ 等待穩定
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 2a] Holding ground position for {TAKEOFF_WAIT_S:.1f} s (PX4 takeoff ramp)...")
rospy.sleep(TAKEOFF_WAIT_S)

climb_from = (ground[0], ground[1], get_altitude())     # 從目前高度開始
climb_to   = (ground[0], ground[1], TARGET_ALT_M)
T_climb    = line_duration(climb_from, climb_to, CLIMB_PEAK_MPS, min_dur=CLIMB_MIN_S)
rospy.loginfo(f"\n[Step 2b] Smooth climb {climb_from[2]:.2f} m → {TARGET_ALT_M:.2f} m "
              f"(peak ≤ {CLIMB_PEAK_MPS} m/s, feedforward: p + v + a)")
run_trajectory(line_segment(climb_from, climb_to, T_climb, ground_yaw), T_climb, "climb")

rospy.loginfo(f"\n[Step 2c] Stable hover at {TARGET_ALT_M} m...")
rospy.loginfo(f"  Waiting (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")
stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()
while not rospy.is_shutdown():
    now, alt = rospy.Time.now(), get_altitude()
    alt_err  = TARGET_ALT_M - alt
    stable_since = (stable_since or now) if abs(alt_err) < ALT_TOL else None
    if (now - last_log).to_sec() > 1.0:
        s = (now - stable_since).to_sec() if stable_since else 0.0
        rospy.loginfo(f"  alt={alt:.2f} m (err={alt_err:+.2f})  "
                      f"stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
        last_log = now
    if stable_since and (now - stable_since).to_sec() >= HOVER_STABLE_TIME:
        rospy.loginfo(f"  Stable at {alt:.2f} m.")
        break
    if (now - t_phase).to_sec() > 25.0:
        rospy.logwarn(f"  Timeout – alt={alt:.2f} m, continuing.")
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 3 – Hover hold, record path origin
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover hold for {HOVER_PHASE_DUR:.0f} s (recording path origin)...")
rospy.sleep(HOVER_PHASE_DUR)
cx, cy, _ = get_xyz()
rospy.loginfo(f"  Path origin: ENU ({cx:.2f}, {cy:.2f}) m")

# 方形頂點（以 cx, cy 為中心）
S = SQUARE_SIDE_M / 2.0
if CLOCKWISE:
    verts  = [(cx + S, cy - S), (cx - S, cy - S), (cx - S, cy + S), (cx + S, cy + S)]
    labels = ['SE', 'SW', 'NW', 'NE']
else:
    verts  = [(cx + S, cy - S), (cx + S, cy + S), (cx - S, cy + S), (cx - S, cy - S)]
    labels = ['SE', 'NE', 'NW', 'SW']
for k, (vx, vy) in enumerate(verts):
    rospy.loginfo(f"  v[{k}] {labels[k]}: ({vx:.3f}, {vy:.3f}) m")

def vert3(i):
    return (verts[i][0], verts[i][1], TARGET_ALT_M)

def seg_yaw(src_i, dst_i):
    if not YAW_TRACK_PATH:
        return 0.0
    return math.atan2(verts[dst_i][1] - verts[src_i][1], verts[dst_i][0] - verts[src_i][0])

# ─────────────────────────────────────────────────────────────
# Step 4 – 移動至 v[0]（最小 jerk）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Approach v[0] ({labels[0]})")
centre   = (cx, cy, TARGET_ALT_M)
init_yaw = seg_yaw(0, 1)
T_app    = line_duration(centre, vert3(0), SEG_PEAK_SPEED_MPS, min_dur=3.0)
run_trajectory(line_segment(centre, vert3(0), T_app, init_yaw), T_app, f"centre → {labels[0]}")
hold(vert3(0), init_yaw, VERTEX_DWELL_S, f"{labels[0]} dwell")

# ─────────────────────────────────────────────────────────────
# Step 5 – 方形軌跡
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Square trajectory ({NUM_LAPS} lap(s))  ~{_LAP_DUR_S:.0f} s/lap")
for lap in range(NUM_LAPS):
    rospy.loginfo(f"\n  ══ Lap {lap + 1}/{NUM_LAPS} ══")
    for seg in range(4):
        src_i, dst_i = seg, (seg + 1) % 4
        yaw      = seg_yaw(src_i, dst_i)
        next_yaw = seg_yaw(dst_i, (dst_i + 1) % 4)
        run_trajectory(line_segment(vert3(src_i), vert3(dst_i), _SEG_DUR_S, yaw), _SEG_DUR_S,
                       f"{labels[src_i]} → {labels[dst_i]}  yaw={math.degrees(yaw):.0f}°")
        hold(vert3(dst_i), next_yaw, VERTEX_DWELL_S,
             f"{labels[dst_i]} dwell, yaw → {math.degrees(next_yaw):.0f}°")
rospy.loginfo(f"\n  ✓ Square trajectory complete ({NUM_LAPS} lap(s)).")

# ─────────────────────────────────────────────────────────────
# Step 6 – 返回中心懸停 5 s
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Return to centre ({cx:.2f}, {cy:.2f}) m...")
T_ret = line_duration(vert3(0), centre, SEG_PEAK_SPEED_MPS, min_dur=3.0)
run_trajectory(line_segment(vert3(0), centre, T_ret, seg_yaw(0, 1)), T_ret, f"{labels[0]} → centre")
hold(centre, seg_yaw(0, 1), 5.0, "centre hold")

# ─────────────────────────────────────────────────────────────
# Land
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Land] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
# 確認已離開 OFFBOARD 才停止 Timer，否則會先觸發 offboard lost → Position mode
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'AUTO.LAND':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  AUTO.LAND not confirmed, retrying...")
        set_mode('AUTO.LAND')
        break
    rate.sleep()
_sp_timer.shutdown()
rospy.sleep(15.0)
rospy.loginfo("Done.")
