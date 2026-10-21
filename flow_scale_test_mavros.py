#!/usr/bin/env python3
"""
Optical-flow scale test (MAVROS / ROS1 Noetic)
===============================================
起飛到 1 m 懸停 → 沿起飛時的機頭方向往前飛 FORWARD_DIST_M（預設 1 m）→ 懸停 → 降落。
比較「EKF 估測的移動距離」與「捲尺量到的實際距離」，算出 SENS_FLOW_SCALE。

■ 測試方法
  1. 起飛前在機身正下方（光流模組下方）的地板貼一個標記 A，機頭朝要飛的方向
     （前方至少留 2 m 空間）。
  2. 執行本腳本。全程 yaw 固定為起飛航向（不轉 yaw，排除轉向造成的光流誤差）。
  3. 降落上鎖後，在機身正下方貼標記 B，量 A → B 的水平距離。
  4. 腳本最後會詢問量到的距離，並算出建議的 SENS_FLOW_SCALE：
        新值 = 目前值 × 實際距離 / EKF 估測距離
     （EKF 估測距離取「解鎖前」到「上鎖後」的估測位置差，與 A → B 對應）
  也會印出懸停段的估測位移（前 / 後各取 MEASURE_S 秒平均），以及與指令 1 m 的比較，
  方便確認控制器是否有確實飛到 setpoint。

■ 注意
  - 位置 / 速度 / 加速度前饋 + yaw（setpoint_raw/local），最小 jerk 曲線，峰值 PEAK_SPEED_MPS。
  - 途中若離開 OFFBOARD 或被上鎖：立即停止送 setpoint 以外的動作，不再切模式，交給飛手。
  - SENS_FLOW_SCALE 需要 7e6a6d4 之後的韌體；找不到參數時以 1.0 計算。
  - 建議 EKF2_MULTI_IMU = 0、SENS_IMU_MODE = 1（避免 EKF 實例切換造成航向 / 位置跳動）。

執行方式：
  python3 flow_scale_test_mavros.py  (ROS 環境已 source，MAVROS 已連線)
"""

import sys
import math
import threading

import rospy
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ParamValue, PositionTarget
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
    ParamGet,    ParamGetRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 1.0    # 懸停高度 (m)
FORWARD_DIST_M    = 1.0    # 往前飛的距離 (m)，沿起飛時的機頭方向
PEAK_SPEED_MPS    = 0.25   # 往前飛的最小 jerk 曲線峰值速度 (m/s)
HOVER_BEFORE_S    = 6.0    # 往前飛之前的懸停時間 (s)
HOVER_AFTER_S     = 6.0    # 往前飛之後的懸停時間 (s)
MEASURE_S         = 3.0    # 懸停段最後幾秒取位置平均 (s)

ALT_TOL           = 0.15   # 穩定懸停高度容差 (m)
HOVER_STABLE_TIME = 3.0    # 穩定判定時間 (s)
ALT_WAIT_TIMEOUT  = 25.0   # 等待高度穩定的上限 (s)

# 平滑起飛（與 *_ff_smooth_mavros.py 相同）
CLIMB_PEAK_MPS    = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S       = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S  = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)

CTRL_HZ           = 20     # setpoint 發布頻率（經 SiK 數傳時建議 10）
LOG_PERIOD_S      = 0.5    # 狀態印出間隔 (s)
LAND_WAIT_S       = 30.0   # 等待落地自動上鎖的上限 (s)
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × L / T

TYPE_MASK_PVA_YAW = PositionTarget.IGNORE_YAW_RATE
ZERO3 = (0.0, 0.0, 0.0)

# ─────────────────────────────────────────────────────────────
# State
# ─────────────────────────────────────────────────────────────
_vehicle_state = State()
_local_pose    = PoseStamped()
_pose_received = threading.Event()


def _cb_state(msg):
    global _vehicle_state
    _vehicle_state = msg


def _cb_pose(msg):
    global _local_pose
    _local_pose = msg
    _pose_received.set()


def get_xyz():
    p = _local_pose.pose.position
    return p.x, p.y, p.z


def get_yaw():
    """目前 yaw（ENU，rad）。"""
    o = _local_pose.pose.orientation
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[2]


def make_target(p, v=ZERO3, a=ZERO3, yaw_rad=0.0):
    """ENU PositionTarget：位置 + 速度 + 加速度 + yaw（MAVROS 轉成 NED 送給 PX4）。"""
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


_sp_lock = threading.Lock()
_sp_msg  = [None]


def set_sp(msg):
    with _sp_lock:
        _sp_msg[0] = msg


def _timer_cb(_event):
    with _sp_lock:
        msg = _sp_msg[0]
    if msg is not None:
        msg.header.stamp = rospy.Time.now()
        try:
            sp_pub.publish(msg)
        except rospy.ROSException:
            pass   # 節點關閉中


def min_jerk(t, T):
    """最小 jerk 曲線 s(t)∈[0,1]：回傳 (s, ṡ, s̈)，起訖速度與加速度皆為 0。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    u = t / T
    return (u**3 * (10.0 - 15.0 * u + 6.0 * u * u),
            30.0 * u * u * (1.0 - u) ** 2 / T,
            60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T))


# ─────────────────────────────────────────────────────────────
# MAVROS services
# ─────────────────────────────────────────────────────────────

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


def param_get(name):
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
# Flight helpers
# ─────────────────────────────────────────────────────────────
class Aborted(Exception):
    """離開 OFFBOARD / 被上鎖：交給飛手 / failsafe，不再切模式。"""


def check_flight():
    if _vehicle_state.mode != 'OFFBOARD' or not _vehicle_state.armed:
        raise Aborted(f"mode={_vehicle_state.mode}, armed={_vehicle_state.armed}")


def run(fn, duration, title):
    """以 fn(t) -> (p, v, a) 送出 setpoint（yaw 固定為起飛航向）直到 duration。"""
    rospy.loginfo(f"  ── {title} ({duration:.1f} s) ──")
    t0 = last_log = rospy.Time.now()
    while not rospy.is_shutdown():
        check_flight()
        now = rospy.Time.now()
        t = min((now - t0).to_sec(), duration)
        p, v, a = fn(t)
        set_sp(make_target(p, v, a, yaw_rad=ground_yaw))
        if (now - last_log).to_sec() >= LOG_PERIOD_S:
            x, y, z = get_xyz()
            rospy.loginfo(f"    {t:5.1f}s  cmd=({p[0]:6.2f},{p[1]:6.2f},{p[2]:4.2f})  "
                          f"est=({x:6.2f},{y:6.2f},{z:4.2f})  "
                          f"err={math.hypot(p[0] - x, p[1] - y):.2f} m  |v_ff|={math.hypot(v[0], v[1]):.2f}")
            last_log = now
        if t >= duration:
            return
        rate.sleep()


def hold_and_average(p, duration, avg_s, title):
    """在 p 懸停 duration 秒，回傳最後 avg_s 秒的平均估測位置 (x, y, z)。"""
    rospy.loginfo(f"  ── {title} ({duration:.1f} s, average last {avg_s:.0f} s) ──")
    set_sp(make_target(p, yaw_rad=ground_yaw))
    t0 = last_log = rospy.Time.now()
    acc = []
    while not rospy.is_shutdown():
        check_flight()
        now = rospy.Time.now()
        t = (now - t0).to_sec()
        if t >= duration - avg_s:
            acc.append(get_xyz())
        if (now - last_log).to_sec() >= 1.0:
            x, y, z = get_xyz()
            rospy.loginfo(f"    {t:4.1f}s  est=({x:6.2f},{y:6.2f},{z:4.2f})  "
                          f"err={math.hypot(p[0] - x, p[1] - y):.2f} m")
            last_log = now
        if t >= duration:
            break
        rate.sleep()
    n = max(1, len(acc))
    return tuple(sum(q[k] for q in acc) / n for k in range(3))


def along_lateral(d):
    """位移 d (ENU xy) 分解成沿起飛機頭方向 / 向左的分量。"""
    c, s = math.cos(ground_yaw), math.sin(ground_yaw)
    return d[0] * c + d[1] * s, -d[0] * s + d[1] * c


# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('flow_scale_test', anonymous=False)
rate = rospy.Rate(CTRL_HZ)
rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

rospy.loginfo("Waiting for local position...")
while not rospy.is_shutdown() and not _pose_received.is_set():
    rate.sleep()
rospy.sleep(1.0)
# 起飛點：取 1 s 平均，作為「標記 A」對應的估測位置
_samples = []
for _ in range(CTRL_HZ):
    _samples.append(get_xyz())
    rate.sleep()
ground     = tuple(sum(q[k] for q in _samples) / len(_samples) for k in range(3))
ground_yaw = get_yaw()
fwd        = (math.cos(ground_yaw), math.sin(ground_yaw))
set_sp(make_target(ground, yaw_rad=ground_yaw))
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_cb)

rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
scale_now = param_get('SENS_FLOW_SCALE')
if scale_now is None or scale_now <= 0.0:
    rospy.logwarn("  SENS_FLOW_SCALE not found (old firmware?) → assuming 1.0")
    scale_now = 1.0
T_fwd = MIN_JERK_PEAK * FORWARD_DIST_M / PEAK_SPEED_MPS
rospy.loginfo(f"  Connected. mode={_vehicle_state.mode}")
rospy.loginfo(f"  Ground (mark A): ENU ({ground[0]:.3f}, {ground[1]:.3f}, {ground[2]:.3f}) m, "
              f"heading {math.degrees(ground_yaw):.1f}° (ENU)")
rospy.loginfo(f"  Plan: hover {TARGET_ALT_M:.1f} m → forward {FORWARD_DIST_M:.2f} m in {T_fwd:.1f} s "
              f"(peak {PEAK_SPEED_MPS:.2f} m/s) → hover → land.  SENS_FLOW_SCALE = {scale_now:.3f}")

rospy.loginfo("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
for _n in ('PFA_DES_ROLL', 'PFA_DES_PITCH'):
    rospy.loginfo(f"  {_n} = 0 ... {'OK' if param_set(_n, 0.0) else 'FAILED'}")
tkf_byp = param_get('PFA_TKF_BYP')
spool_s = param_get('COM_SPOOLUP_TIME')
ramp_s  = param_get('MPC_TKO_RAMP_T')
if tkf_byp is not None and int(round(tkf_byp)) == 0:
    rospy.logwarn("  WARNING: PFA_TKF_BYP = 0 → 推力上限會卡在 0.1，請改回 1。")
TAKEOFF_WAIT_S = (1.0 if spool_s is None else spool_s) + (3.0 if ramp_s is None else ramp_s) \
    + TAKEOFF_SETTLE_S

rospy.loginfo("\n[Step 1] Streaming ground setpoints (5 s) → OFFBOARD → arm")
rospy.sleep(5.0)
set_mode('OFFBOARD')
t_w = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t_w).to_sec() > 5.0:
        rospy.logwarn("  OFFBOARD not confirmed.")
        break
    rate.sleep()
arm_vehicle()
t_w = rospy.Time.now()
while not rospy.is_shutdown() and not _vehicle_state.armed:
    if (rospy.Time.now() - t_w).to_sec() > 10.0:
        rospy.logerr("  ERROR: Failed to arm (check 'Preflight Fail' messages).")
        _sp_timer.shutdown()
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  Armed.")

result = None
try:
    # ── Step 2：平滑起飛 ──
    rospy.loginfo(f"\n[Step 2] Takeoff ramp {TAKEOFF_WAIT_S:.1f} s → smooth climb to {TARGET_ALT_M} m")
    t_w = rospy.Time.now()
    while (rospy.Time.now() - t_w).to_sec() < TAKEOFF_WAIT_S:
        check_flight()
        rate.sleep()
    z0 = get_xyz()[2]
    dz = TARGET_ALT_M - z0
    T_cl = max(CLIMB_MIN_S, MIN_JERK_PEAK * abs(dz) / CLIMB_PEAK_MPS)

    def _climb(t):
        s, sd, sdd = min_jerk(t, T_cl)
        return (ground[0], ground[1], z0 + s * dz), (0.0, 0.0, sd * dz), (0.0, 0.0, sdd * dz)
    run(_climb, T_cl, "climb")
    p_start = (ground[0], ground[1], TARGET_ALT_M)
    set_sp(make_target(p_start, yaw_rad=ground_yaw))

    stable_since, t_w = None, rospy.Time.now()
    while not rospy.is_shutdown():
        check_flight()
        now = rospy.Time.now()
        stable_since = (stable_since or now) if abs(TARGET_ALT_M - get_xyz()[2]) < ALT_TOL else None
        if stable_since and (now - stable_since).to_sec() >= HOVER_STABLE_TIME:
            rospy.loginfo(f"  Stable at {get_xyz()[2]:.2f} m.")
            break
        if (now - t_w).to_sec() > ALT_WAIT_TIMEOUT:
            rospy.logwarn(f"  Timeout – alt={get_xyz()[2]:.2f} m, continuing.")
            break
        rate.sleep()

    # ── Step 3：懸停 → 往前 FORWARD_DIST_M → 懸停 ──
    rospy.loginfo("\n[Step 3] Hover / forward / hover")
    est0 = hold_and_average(p_start, HOVER_BEFORE_S, MEASURE_S, "hover before")
    p_end = (p_start[0] + FORWARD_DIST_M * fwd[0], p_start[1] + FORWARD_DIST_M * fwd[1], TARGET_ALT_M)

    def _forward(t):
        s, sd, sdd = min_jerk(t, T_fwd)
        return (tuple(a + s * (b - a) for a, b in zip(p_start, p_end)),
                tuple(sd * (b - a) for a, b in zip(p_start, p_end)),
                tuple(sdd * (b - a) for a, b in zip(p_start, p_end)))
    run(_forward, T_fwd, f"forward {FORWARD_DIST_M:.2f} m")
    est1 = hold_and_average(p_end, HOVER_AFTER_S, MEASURE_S, "hover after")
    result = (est0, est1)
except Aborted as e:
    rospy.logerr(f"\n  ✗ Aborted: {e} → 不再切換模式，交給飛手 / failsafe。")
    _sp_timer.shutdown()
    sys.exit(1)

# ── Land ──
rospy.loginfo("\n[Land] AUTO.LAND")
set_mode('AUTO.LAND')
t_w = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'AUTO.LAND':
    if (rospy.Time.now() - t_w).to_sec() > 5.0:
        set_mode('AUTO.LAND')
        break
    rate.sleep()
_sp_timer.shutdown()
rospy.loginfo("  Waiting for touchdown / auto disarm...")
t_w = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.armed:
    if (rospy.Time.now() - t_w).to_sec() > LAND_WAIT_S:
        rospy.logwarn("  Still armed after landing timeout; using current position.")
        break
    rate.sleep()
rospy.sleep(1.0)
landed = get_xyz()

# ─────────────────────────────────────────────────────────────
# Report
# ─────────────────────────────────────────────────────────────
est0, est1 = result
d_hover = (est1[0] - est0[0], est1[1] - est0[1])
d_total = (landed[0] - ground[0], landed[1] - ground[1])
ah, lh = along_lateral(d_hover)
at, lt = along_lateral(d_total)
est_total = math.hypot(*d_total)
rospy.loginfo("\n══════════ Result ══════════")
rospy.loginfo(f"  Command               : forward {FORWARD_DIST_M:.3f} m")
rospy.loginfo(f"  EKF hover → hover     : {math.hypot(*d_hover):.3f} m  "
              f"(forward {ah:+.3f}, left {lh:+.3f})")
rospy.loginfo(f"  EKF mark A → mark B   : {est_total:.3f} m  "
              f"(forward {at:+.3f}, left {lt:+.3f})   ← 與捲尺量的 A → B 比較")
rospy.loginfo(f"  Current SENS_FLOW_SCALE = {scale_now:.3f}")
rospy.loginfo("  量地板上標記 A → B 的水平距離，輸入後計算建議值（直接 Enter 跳過）。")

try:
    s = input("  實際距離 A → B (m): ").strip() if sys.stdin.isatty() else ""
except EOFError:
    s = ""
if s:
    try:
        real = float(s)
        new_scale = scale_now * real / est_total
        rospy.loginfo(f"  實際 / 估測 = {real:.3f} / {est_total:.3f} = {real / est_total:.3f}")
        rospy.loginfo(f"  建議 SENS_FLOW_SCALE = {scale_now:.3f} × {real / est_total:.3f} = {new_scale:.3f}")
        if not 0.5 <= new_scale <= 3.0:
            rospy.logwarn("  超出參數範圍 0.5–3.0，請確認量測（標記位置、是否量到水平距離）。")
    except (ValueError, ZeroDivisionError):
        rospy.logwarn("  無法計算（輸入不是數字或估測距離為 0）。")
rospy.loginfo("Done.")
