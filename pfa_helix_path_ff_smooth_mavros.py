#!/usr/bin/env python3
"""
PFA Helix Path Tracking with Feedforward (MAVROS / ROS1 Noetic)
=================================================================
■ 平滑起飛版（pfa_helix_path_ff_mavros.py 的平滑起飛版本，起飛後的流程不變）
  原版解鎖前就送 z = TARGET_ALT_M：起飛緩升（PFA_TKF_BYP=1）結束、切到位置控制的瞬間
  高度誤差 ≈ TARGET_ALT_M，P 項把推力推到接近滿載 → 往上衝 → 電流暴增、外部電源壓降
  → 推力不足又掉下來。本版：
    1. 解鎖前 setpoint = 目前地面位置與機頭方向（高度誤差 0，起飛時 yaw 不轉）
    2. 解鎖後維持地面位置，等 PX4 起飛緩升結束
       （COM_SPOOLUP_TIME + MPC_TKO_RAMP_T，從飛控讀取，再加 TAKEOFF_SETTLE_S）
    3. 以最小 jerk 曲線從地面高度爬升到 TARGET_ALT_M（峰值 CLIMB_PEAK_MPS），含 v / a 前饋
    4. 高度穩定後，以最小 jerk 曲線把 yaw 由起飛航向平滑轉到 0°（原版懸停 yaw），
       之後的軌跡流程與原版相同
    5. 軌跡中心改用懸停 setpoint（地面 XY），不用量測位置，避免 setpoint 跳一段穩態誤差
  注意：OFFBOARD 下必須 PFA_TKF_BYP = 1，否則推力上限會卡在 0.1（啟動時會檢查）。

Task: 追蹤螺旋上升軌跡，roll=pitch=0°，yaw 跟隨切線方向，
      同時送出位置 + 速度 + 加速度前饋。pfa_helix_path_mavros.py 的前饋版本。

■ 為什麼要前饋
  只送位置時 trajectory_setpoint.velocity/acceleration = NaN → pfa_pos_control
  清成 0，D 項以 0 為速度參考一直煞車 → 固定落後 (Kv/Kp)·速度（水平與爬升都會落後）。
  送出 v_ref、a_ref（含向心加速度、爬升速度）後誤差動態為 ë + Kv·ė + Kp·e = 0。
  （前饋無法消除重心偏移等固定干擾造成的穩態誤差，那需要積分項。）

■ 軌跡（ENU，俯視逆時針，以懸停位置為中心）
  x = hx + R·cos(ωτ),  y = hy + R·sin(ωτ),  z = h₀ + CLIMB_RATE_MPS·τ
  τ(t)：時間扭曲，前後各 RAMP_S 秒平滑加減速（smoothstep，加速度也連續）（幾何路徑不變，起停速度為 0）
  前饋：v = dp/dτ·τ̇，a = d²p/dτ²·τ̇² + dp/dτ·τ̈
  切線速度 v_tan = 2πR / T_rev，向心加速度 a_c = v_tan² / R
  yaw = 切線方向 (ωτ + 90°，ENU)

■ 控制鏈
  set_sp() → rospy.Timer (CTRL_HZ) → /mavros/setpoint_raw/local (PositionTarget)
  → SET_POSITION_TARGET_LOCAL_NED (p + v + a + yaw) → pfa_pos_control → pfa_att_control

執行方式：
  python3 pfa_helix_path_ff_smooth_mavros.py  (ROS 環境已 source)
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
    ParamGet,    ParamGetRequest,
    ParamPull,   ParamPullRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M        = 1.0    # 螺旋起始高度 (m，ENU z+)
HELIX_RADIUS_M      = 1.5    # 螺旋半徑 (m)
CLIMB_RATE_MPS      = 0.3    # 垂直爬升速率 (m/s，等速段)
REV_PERIOD_S        = 8.0    # 每圈時間 (s/rev，等速段)
NUM_REVOLUTIONS     = 3      # 總圈數
RAMP_S              = 3.0    # 起始加速 / 結束減速時間 (s)
YAW_TRACK_PATH      = True   # True: yaw 跟切線; False: yaw=0° 固定

HOVER_PHASE_DUR     = 8.0
ALT_TOL             = 0.15
HOVER_STABLE_TIME   = 3.0

# 平滑起飛
CLIMB_PEAK_MPS    = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S       = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S  = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)
YAW_PEAK_RATE_DPS = 45.0   # 爬升後由起飛航向轉到 yaw = 0° 的峰值角速度 (°/s)
APPROACH_PEAK_MPS   = 0.5    # 移動到螺旋起點的峰值速度 (m/s)
APPROACH_MIN_S      = 4.0    # 移動到螺旋起點的最短時間 (s)
YAW_SETTLE_S        = 2.0    # 起點停留，讓 yaw 轉到切線方向 (s)
END_HOLD_S          = 10.0   # 終點保持時間 (s)

CTRL_HZ           = 20     # Timer 發布頻率（經 SiK 數傳時建議降到 10）
LOG_PERIOD_S      = 0.5    # 追蹤狀態印出間隔 (s)
PX4_SPEED_XY_MAX  = 1.0    # pfa_pos_control 的 _speed_xy_max (m/s)，水平速度參考上限
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × L / T

# PositionTarget.type_mask：位置、速度、加速度、yaw 都使用；只忽略 yaw_rate
# （pfa_pos_control / pfa_att_control 目前不讀 trajectory_setpoint.yawspeed）
TYPE_MASK_PVA_YAW = PositionTarget.IGNORE_YAW_RATE

ZERO3 = (0.0, 0.0, 0.0)

# Derived
OMEGA         = 2.0 * math.pi / REV_PERIOD_S
_PATH_S       = NUM_REVOLUTIONS * REV_PERIOD_S          # 等速時的路徑時間 (s)
TOTAL_DUR_S   = _PATH_S + RAMP_S                        # 含加減速的總時間 (s)
TOTAL_CLIMB_M = CLIMB_RATE_MPS * _PATH_S
FINAL_ALT_M   = TARGET_ALT_M + TOTAL_CLIMB_M
V_TAN_MPS     = OMEGA * HELIX_RADIUS_M
A_CENTRIPETAL = V_TAN_MPS ** 2 / HELIX_RADIUS_M


def helix_path(hx, hy):
    """等速螺旋 path(τ) -> (p, dp/dτ, d²p/dτ², yaw)。"""
    def path(tau):
        th   = OMEGA * tau
        c, s = math.cos(th), math.sin(th)
        p    = (hx + HELIX_RADIUS_M * c, hy + HELIX_RADIUS_M * s, TARGET_ALT_M + CLIMB_RATE_MPS * tau)
        dp   = (-HELIX_RADIUS_M * OMEGA * s, HELIX_RADIUS_M * OMEGA * c, CLIMB_RATE_MPS)
        ddp  = (-HELIX_RADIUS_M * OMEGA ** 2 * c, -HELIX_RADIUS_M * OMEGA ** 2 * s, 0.0)
        yaw  = math.atan2(c, -s) if YAW_TRACK_PATH else 0.0
        return p, dp, ddp, yaw
    return path


# ─────────────────────────────────────────────────────────────
# Global state
# ─────────────────────────────────────────────────────────────
_vehicle_state = State()
_local_pose    = PoseStamped()
_imu_data      = Imu()


def _cb_state(msg): global _vehicle_state; _vehicle_state = msg
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


def ramp_time(t, path_dur, ramp):
    """時間扭曲 τ(t)：τ̇ 在前後 ramp 秒內以 smoothstep 3u²−2u³ 平滑 0↔1，中段 τ̇ = 1。

    回傳 (τ, τ̇, τ̈)；τ ∈ [0, path_dur]，總時間 = path_dur + ramp。
    幾何路徑不變，只改變沿路徑的速度 → 速度與加速度前饋都連續、起停時為 0。
    （加減速段的位移 = ramp/2，與線性加速相同，所以總時間不變。）
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


def warped(path_fn, path_dur, ramp):
    """把等速路徑 path_fn(τ) -> (p, dp/dτ, d²p/dτ², yaw) 套上 ramp_time 成為 fn(t)。"""
    def fn(t):
        tau, td, tdd = ramp_time(t, path_dur, ramp)
        p, dp, ddp, yaw = path_fn(tau)
        v = tuple(k * td for k in dp)
        a = tuple(kk * td * td + k * tdd for k, kk in zip(dp, ddp))
        return p, v, a, yaw
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
# 平滑起飛 helpers
# ─────────────────────────────────────────────────────────────
_pose_received = threading.Event()


def get_yaw():
    """目前 yaw（ENU，rad），取自 /mavros/local_position/pose。"""
    o = _local_pose.pose.orientation
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[2]


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


def _tk_min_jerk(t, T):
    """最小 jerk 曲線 s(t)∈[0,1]：回傳 (s, ṡ, s̈)，起訖速度與加速度皆為 0。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    u = t / T
    return (u**3 * (10.0 - 15.0 * u + 6.0 * u * u),
            30.0 * u * u * (1.0 - u) ** 2 / T,
            60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T))


def _tk_wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def smooth_climb(xy, z0, z1, yaw_rad):
    """在 xy 以最小 jerk 曲線由 z0 爬升到 z1（含 v / a 前饋），yaw 固定。"""
    dz = z1 - z0
    T  = max(CLIMB_MIN_S, 1.875 * abs(dz) / CLIMB_PEAK_MPS)
    rospy.loginfo(f"  climb {z0:.2f} m → {z1:.2f} m in {T:.1f} s "
                  f"(peak {1.875 * abs(dz) / T:.2f} m/s, feedforward: p + v + a)")
    t0, last_log = rospy.Time.now(), rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t   = (now - t0).to_sec()
        s, sd, sdd = _tk_min_jerk(t, T)
        set_sp(make_target((xy[0], xy[1], z0 + s * dz), (0.0, 0.0, sd * dz),
                           (0.0, 0.0, sdd * dz), yaw_rad=yaw_rad))
        if (now - last_log).to_sec() >= 0.5:
            rospy.loginfo(f"    {t:4.1f}s  alt_cmd={z0 + s * dz:.2f} m  alt={get_altitude():.2f} m")
            last_log = now
        if t >= T:
            break
        rate.sleep()
    set_sp(make_target((xy[0], xy[1], z1), yaw_rad=yaw_rad))


def smooth_yaw(p, yaw0, yaw1):
    """在 p 原地以最小 jerk 曲線由 yaw0 轉到 yaw1（最短方向），再停 0.5 s。"""
    d = _tk_wrap_pi(yaw1 - yaw0)
    if abs(d) > math.radians(1.0):
        T = max(1.0, 1.875 * math.degrees(abs(d)) / YAW_PEAK_RATE_DPS)
        rospy.loginfo(f"  yaw {math.degrees(yaw0):.0f}° → {math.degrees(yaw1):.0f}° in {T:.1f} s")
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - t0).to_sec()
            s, _, _ = _tk_min_jerk(t, T)
            set_sp(make_target(p, yaw_rad=_tk_wrap_pi(yaw0 + d * s)))
            if t >= T:
                break
            rate.sleep()
    set_sp(make_target(p, yaw_rad=yaw1))
    rospy.sleep(0.5)

# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('pfa_helix_ff_smooth', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

sp_pub = rospy.Publisher(
    '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

rospy.loginfo("Waiting for local position...")
while not rospy.is_shutdown() and not _pose_received.is_set():
    rate.sleep()
# 解鎖前 setpoint = 目前地面位置與機頭方向（高度誤差 0），而不是 TARGET_ALT_M
ground     = get_xyz()
ground_yaw = get_yaw()
set_sp(make_target(ground, yaw_rad=ground_yaw))   # 初始 setpoint（Timer 啟動前先設定）
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_cb)
rospy.loginfo(f"  Ground position: ENU ({ground[0]:.2f}, {ground[1]:.2f}, {ground[2]:.2f}) m  "
              f"yaw={math.degrees(ground_yaw):.1f}°")

rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected. mode={_vehicle_state.mode}")
rospy.loginfo(f"  Helix (feedforward: p + v + a): R={HELIX_RADIUS_M} m, T={REV_PERIOD_S} s/rev, "
              f"{NUM_REVOLUTIONS} revs, climb={CLIMB_RATE_MPS} m/s (+{RAMP_S:.0f} s ramp)")
rospy.loginfo(f"  v_tan={V_TAN_MPS:.2f} m/s, a_centripetal={A_CENTRIPETAL:.3f} m/s²")
rospy.loginfo(f"  Alt: {TARGET_ALT_M:.1f} m → {FINAL_ALT_M:.1f} m in {TOTAL_DUR_S:.0f} s")
warn_speed(V_TAN_MPS, "helix")
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
rospy.loginfo("\n[Step 1] Streaming ground-position setpoints (5 s)...")
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
# Step 2a/2b – 等 PX4 起飛緩升結束 → 最小 jerk 爬升（含前饋）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 2a] Holding ground position for {TAKEOFF_WAIT_S:.1f} s (PX4 takeoff ramp)...")
rospy.sleep(TAKEOFF_WAIT_S)
rospy.loginfo(f"\n[Step 2b] Smooth climb (peak ≤ {CLIMB_PEAK_MPS} m/s)")
smooth_climb(ground, get_altitude(), TARGET_ALT_M, ground_yaw)
hover_p = (ground[0], ground[1], TARGET_ALT_M)

# ─────────────────────────────────────────────────────────────
# Step 2c – Stable hover at TARGET_ALT_M
# ─────────────────────────────────────────────────────────────
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

rospy.loginfo("\n[Step 2d] Smooth yaw from take-off heading to 0°")
smooth_yaw(hover_p, ground_yaw, 0.0)

# ─────────────────────────────────────────────────────────────
# Step 3 – Hover hold, record path origin
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover hold for {HOVER_PHASE_DUR:.0f} s (recording path origin)...")
rospy.sleep(HOVER_PHASE_DUR)
cx, cy = hover_p[0], hover_p[1]   # 以懸停 setpoint 為中心，避免 setpoint 跳一段穩態誤差
rospy.loginfo(f"  Measured hover position: ENU ({get_xyz()[0]:.2f}, {get_xyz()[1]:.2f}) m")
rospy.loginfo(f"  Path origin: ENU ({cx:.2f}, {cy:.2f}) m")

path   = helix_path(cx, cy)
centre = (cx, cy, TARGET_ALT_M)
p0, _, _, yaw0 = path(0.0)

# ─────────────────────────────────────────────────────────────
# Step 4 – 接近螺旋起點 (hx+R, hy)
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Approaching helix start ({p0[0]:.2f}, {p0[1]:.2f}) m...")
T_app = line_duration(centre, p0, APPROACH_PEAK_MPS, min_dur=APPROACH_MIN_S)
run_trajectory(line_segment(centre, p0, T_app, yaw0), T_app, "centre → helix start")
hold(p0, yaw0, YAW_SETTLE_S, "helix start, yaw settle")

# ─────────────────────────────────────────────────────────────
# Step 5 – 螺旋上升
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Helix ascent: {NUM_REVOLUTIONS} revs, "
              f"{TARGET_ALT_M:.1f}→{FINAL_ALT_M:.1f} m, {TOTAL_DUR_S:.0f} s")
p_end, _, _, yaw_end = run_trajectory(warped(path, _PATH_S, RAMP_S), TOTAL_DUR_S, "helix")
rospy.loginfo("\n  Helix ascent complete.")

# ─────────────────────────────────────────────────────────────
# Step 6 – Hold end setpoint
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Holding end point ({p_end[0]:.2f}, {p_end[1]:.2f}, alt={p_end[2]:.2f} m)...")
hold(p_end, yaw_end, END_HOLD_S, "end hold")

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
