#!/usr/bin/env python3
"""
Hover Test – Smooth Climb (MAVROS / ROS 1)
============================================
hover_test_mavros.py 的平滑上升版本：起飛至 0.5 m，維持 30 秒，AUTO.LAND 降落。

■ 為什麼要平滑上升
  原版解鎖前就送 z = TARGET_ALT_M。pfa_pos_control 的起飛緩升（PFA_TKF_BYP=1）
  結束、切到正常位置控制的瞬間，高度誤差 ≈ TARGET_ALT_M，P 項把推力推到接近
  滿載 → 往上衝；電流暴增造成外部電源壓降 → 推力不足又掉下來。

■ 本版流程
  1. 解鎖前 setpoint = 目前地面位置與航向（高度誤差 0，yaw 不會一起飛就轉）
  2. 解鎖後維持地面位置，等 PX4 起飛緩升結束
     （COM_SPOOLUP_TIME + MPC_TKO_RAMP_T，從飛控讀取，再加 TAKEOFF_SETTLE_S）
  3. 以最小 jerk 曲線 s(t) = 10u³ − 15u⁴ + 6u⁵ 從地面高度爬升到 TARGET_ALT_M，
     峰值速度 CLIMB_PEAK_MPS，同時送出速度與加速度前饋
     （/mavros/setpoint_raw/local，PositionTarget）
  4. 懸停 HOVER_DUR_S 秒 → AUTO.LAND，確認進入 AUTO.LAND 後才停止 setpoint
  注意：OFFBOARD 下必須 PFA_TKF_BYP = 1，否則起飛狀態機收不到 want_takeoff，
        推力上限會一直卡在 0.1（啟動時會檢查並警告）。

執行方式：
  python3 hover_test_smooth_mavros.py
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
TARGET_ALT_M     = 0.5    # 懸停高度 (m)
HOVER_DUR_S      = 30.0   # 懸停時間 (s)
CTRL_HZ          = 20     # setpoint 發布頻率（經 SiK 數傳時建議降到 10）

CLIMB_PEAK_MPS   = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S      = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)
MIN_JERK_PEAK    = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × 距離 / 時間

# PositionTarget.type_mask：位置、速度、加速度、yaw 都使用；只忽略 yaw_rate
TYPE_MASK_PVA_YAW = PositionTarget.IGNORE_YAW_RATE

# ─────────────────────────────────────────────────────────────
# State
# ─────────────────────────────────────────────────────────────
_state = State()
_pose  = PoseStamped()
_pose_received = threading.Event()

def _cb_state(msg): global _state; _state = msg
def _cb_pose(msg):
    global _pose
    _pose = msg
    _pose_received.set()

def get_alt(): return _pose.pose.position.z

def get_xyz():
    p = _pose.pose.position
    return p.x, p.y, p.z

def get_yaw():
    o = _pose.pose.orientation
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

# ─────────────────────────────────────────────────────────────
# Shared setpoint (Timer 讀取；主執行緒寫入)
# ─────────────────────────────────────────────────────────────
_sp      = {'p': (0.0, 0.0, 0.0), 'v': (0.0, 0.0, 0.0), 'a': (0.0, 0.0, 0.0), 'yaw': 0.0}
_sp_lock = threading.Lock()

def update_sp(p, v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0), yaw=None):
    """更新 ENU setpoint（位置 / 速度 / 加速度 / yaw），由 Timer 在背景發布。"""
    with _sp_lock:
        _sp['p'], _sp['v'], _sp['a'] = tuple(p), tuple(v), tuple(a)
        if yaw is not None:
            _sp['yaw'] = yaw

def make_target(p, v, a, yaw):
    """ENU PositionTarget；三軸皆為有限值（pfa_pos_control 遇 NaN 會整個向量清 0）。"""
    sp = PositionTarget()
    sp.header.stamp     = rospy.Time.now()
    sp.header.frame_id  = 'map'
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    sp.type_mask        = TYPE_MASK_PVA_YAW
    sp.position.x, sp.position.y, sp.position.z = p
    sp.velocity.x, sp.velocity.y, sp.velocity.z = v
    sp.acceleration_or_force.x, sp.acceleration_or_force.y, sp.acceleration_or_force.z = a
    sp.yaw      = yaw
    sp.yaw_rate = 0.0
    return sp

def _timer_publish(_event):
    with _sp_lock:
        p, v, a, yaw = _sp['p'], _sp['v'], _sp['a'], _sp['yaw']
    sp_pub.publish(make_target(p, v, a, yaw))

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
# Service helpers
# ─────────────────────────────────────────────────────────────
def param_set(name, value):
    try:
        rospy.wait_for_service('/mavros/param/set', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        return svc(req).success
    except Exception as e:
        rospy.logwarn(f"param_set({name}): {e}")
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
    except Exception as e:
        rospy.logwarn(f"param_get({name}): {e}")
        return None

def set_mode(mode):
    try:
        rospy.wait_for_service('/mavros/set_mode', timeout=5)
        svc = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        return svc(SetModeRequest(custom_mode=mode)).mode_sent
    except Exception as e:
        rospy.logwarn(f"set_mode({mode}): {e}")
        return False

def arm():
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        return svc(CommandBoolRequest(value=True)).success
    except Exception as e:
        rospy.logwarn(f"arming: {e}")
        return False

# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('hover_test_smooth', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線與位置估測
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected!  mode={_state.mode}  armed={_state.armed}")

rospy.loginfo("Waiting for local position...")
while not rospy.is_shutdown() and not _pose_received.is_set():
    rate.sleep()

# 解鎖前 setpoint = 目前地面位置與航向（高度誤差 0），而不是 TARGET_ALT_M
ground     = get_xyz()
ground_yaw = get_yaw()
update_sp(ground, yaw=ground_yaw)
rospy.loginfo(f"  Ground position: ENU ({ground[0]:.2f}, {ground[1]:.2f}, {ground[2]:.2f}) m  "
              f"yaw={math.degrees(ground_yaw):.1f}°")

# Timer 在背景執行緒以 CTRL_HZ 持續發布 setpoint
# 確保 arm() / set_mode() 阻塞期間 PX4 不會 timeout
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_publish)

# ─────────────────────────────────────────────────────────────
# Step 0: PFA 姿態參數歸零、讀取起飛緩升參數
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
rospy.loginfo("  PFA_DES_ROLL  ... " + ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH ... " + ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

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
# Step 1: 串流 setpoint（OFFBOARD 前置條件；Timer 已在背景發布）
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 1] Streaming ground-position setpoints for 5 s...")
rospy.sleep(5.0)   # Timer 持續發布，主執行緒只需等待

# ─────────────────────────────────────────────────────────────
# Step 2: 切換 OFFBOARD
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 2] Requesting OFFBOARD mode...")
set_mode('OFFBOARD')   # Timer 在背景持續發布，不中斷

t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_state.mode}")

# ─────────────────────────────────────────────────────────────
# Step 3: 解鎖
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 3] Arming...")
arm()   # 阻塞服務呼叫；Timer 在背景持續發布 setpoint，PX4 不會 timeout

t0 = rospy.Time.now()
while not rospy.is_shutdown() and not _state.armed:
    if (rospy.Time.now() - t0).to_sec() > 10.0:
        rospy.logerr("  ✗ Failed to arm. Check QGC for pre-arm errors.")
        _sp_timer.shutdown()
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  ✓ Armed!")

# ─────────────────────────────────────────────────────────────
# Step 4: 等待 PX4 起飛緩升結束（setpoint 維持在地面位置）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Holding ground position for {TAKEOFF_WAIT_S:.1f} s (PX4 takeoff ramp)...")
rospy.sleep(TAKEOFF_WAIT_S)

# ─────────────────────────────────────────────────────────────
# Step 5: 平滑爬升（最小 jerk 曲線 + 速度 / 加速度前饋）
# ─────────────────────────────────────────────────────────────
z0   = get_alt()                                 # 從目前高度開始（ENU，向上為正）
dz   = TARGET_ALT_M - z0
T_cl = max(CLIMB_MIN_S, MIN_JERK_PEAK * abs(dz) / CLIMB_PEAK_MPS)
rospy.loginfo(f"\n[Step 5] Smooth climb {z0:.2f} m → {TARGET_ALT_M:.2f} m in {T_cl:.1f} s "
              f"(peak {MIN_JERK_PEAK * abs(dz) / T_cl:.2f} m/s, feedforward: p + v + a)")

t0       = rospy.Time.now()
last_log = rospy.Time.now()
while not rospy.is_shutdown():
    now = rospy.Time.now()
    t   = (now - t0).to_sec()
    s, sd, sdd = min_jerk(t, T_cl)
    update_sp((ground[0], ground[1], z0 + s * dz), (0.0, 0.0, sd * dz), (0.0, 0.0, sdd * dz))
    if (now - last_log).to_sec() >= 0.5:
        rospy.loginfo(f"    {t:4.1f}s  alt_cmd={z0 + s * dz:.2f} m  alt={get_alt():.2f} m  "
                      f"vz_ff={sd * dz:+.2f} m/s")
        last_log = now
    if t >= T_cl:
        break
    rate.sleep()
update_sp((ground[0], ground[1], TARGET_ALT_M))   # 爬升結束：保持目標高度，前饋歸零

# ─────────────────────────────────────────────────────────────
# Step 6: 懸停 HOVER_DUR_S 秒
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Hovering at {TARGET_ALT_M} m for {HOVER_DUR_S:.0f} s...")
rospy.loginfo(f"  {'Time':>6}  {'Alt':>6}  {'Err':>6}")
rospy.loginfo("  " + "─" * 24)

t0       = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t0).to_sec()

    if (now - last_log).to_sec() > 2.0:
        rospy.loginfo(f"  {elapsed:6.1f}s  {get_alt():6.2f}m  {TARGET_ALT_M - get_alt():+6.2f}m")
        last_log = now

    if elapsed >= HOVER_DUR_S:
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 7: 降落
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 7] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
# 確認已離開 OFFBOARD 才停止 Timer，否則會先觸發 offboard lost → Position mode
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'AUTO.LAND':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  AUTO.LAND not confirmed, retrying...")
        set_mode('AUTO.LAND')
        break
    rate.sleep()
_sp_timer.shutdown()
rospy.sleep(10.0)
rospy.loginfo("Done.")
