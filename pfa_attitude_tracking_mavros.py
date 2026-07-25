#!/usr/bin/env python3
"""
PFA Attitude Tracking Task (MAVROS / ROS1 Noetic)
==================================================
MAVROS 版本，對應 pfa_attitude_tracking.py (pymavlink 版)

Task: 懸停於高度 1 m，pitch 維持 45 度

控制架構（同 pymavlink 版）：
  Python → PARAM_SET PFA_DES_PITCH = 45°
         → /mavros/setpoint_position/local (ENU, z=+1 m, orientation 帶 yaw)
                         ↓ MAVROS 自動 ENU→NED 轉換
  PX4：  trajectory_setpoint (z=-1 m, yaw=有限值)
                         ↓
         pfa_pos_control  → thrust_body (位置 PD 計算)
                          → pitch_body  (= PFA_DES_PITCH, pass-through)
                         ↓
         pfa_att_control  → 驅動機體到 pitch=45°

與 pymavlink 版的關鍵差異：
  座標系：ENU (z+ = 上，yaw=0 朝東) ← MAVROS 負責轉換到 PX4 的 NED
  Yaw 防 NaN：PoseStamped 的 orientation quaternion 自動帶有限 yaw，
              MAVROS 轉換為 SET_POSITION_TARGET_LOCAL_NED type_mask=position+yaw
  Param：/mavros/param/set ROS service（非 MAVLink PARAM_SET 直送）
  Arm/Mode：/mavros/cmd/arming、/mavros/set_mode services

執行方式：
  rosrun <your_package> pfa_attitude_tracking_mavros.py
  或
  python3 pfa_attitude_tracking_mavros.py  (ROS 環境已 source)
"""

import sys
import math

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamGet,    ParamGetRequest,
    ParamSet,    ParamSetRequest,
    ParamPull,   ParamPullRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 1.0       # 懸停高度 (m，ENU: z+ 方向)
TARGET_PITCH_DEG  = 45.0      # 期望 pitch (deg，正值 = nose up)
TARGET_ROLL_DEG   = 0.0

HOVER_PHASE_DUR   = 8.0       # 水平懸停穩定後等待時間 (s)
PITCH_RAMP_TIME   = 4.0       # pitch 0°→45° 爬升時間 (s)
TRACK_PHASE_DUR   = 30.0      # 維持 pitch=45° 的追蹤時長 (s)
PARAM_STEP_DEG    = 5.0       # 每次 PARAM_SET 的 pitch 步進量 (deg)

ALT_TOL           = 0.15      # 高度容忍範圍 (m)
HOVER_STABLE_TIME = 3.0       # 在容忍範圍內連續 N 秒才確認穩定

CTRL_HZ           = 20        # 控制迴路頻率 (Hz)

# ─────────────────────────────────────────────────────────────
# Global state (updated by ROS subscribers)
# ─────────────────────────────────────────────────────────────
_vehicle_state = State()
_local_pose    = PoseStamped()
_imu_data      = Imu()


def _cb_state(msg):
    global _vehicle_state
    _vehicle_state = msg


def _cb_pose(msg):
    global _local_pose
    _local_pose = msg


def _cb_imu(msg):
    global _imu_data
    _imu_data = msg


# ─────────────────────────────────────────────────────────────
# State accessors
# ─────────────────────────────────────────────────────────────

def get_altitude():
    """ENU z 座標 = 高度 (m，正值朝上)。"""
    return _local_pose.pose.position.z


def get_pitch_deg():
    """
    從 IMU 四元數取得 pitch 角 (度)，正值 = nose up。
    MAVROS imu/data 的方向為 ENU/FLU 座標系，
    tf.euler_from_quaternion 的 pitch 對應物理上的俯仰角，
    與 PX4 NED/FRD 的 pitch 正方向相同。
    """
    o = _imu_data.orientation
    _, pitch_rad, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(pitch_rad)


def get_yaw_rad():
    """從 local_position/pose 取得當前 yaw (rad，ENU 約定)。"""
    o = _local_pose.pose.orientation
    _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return yaw


# ─────────────────────────────────────────────────────────────
# Setpoint helper
# ─────────────────────────────────────────────────────────────

def make_hover_setpoint(alt_m=None, yaw_rad=None):
    """
    ENU 座標系的位置 setpoint (PoseStamped)。

    MAVROS setpoint_position/local plugin 會把此訊息轉成
    SET_POSITION_TARGET_LOCAL_NED (type_mask = position + yaw)，
    因此 trajectory_setpoint.yaw 是有限值，不會觸發 pfa_att_control
    的 _checkAllFinite 零化問題。

    alt_m  : 目標高度 (m)，預設 TARGET_ALT_M
    yaw_rad: 目標 yaw (rad，ENU)，預設維持當前 yaw
    """
    if alt_m is None:
        alt_m = TARGET_ALT_M
    if yaw_rad is None:
        yaw_rad = get_yaw_rad()       # 維持當前 yaw，不強制旋轉

    sp = PoseStamped()
    sp.header.stamp    = rospy.Time.now()
    sp.header.frame_id = 'map'

    sp.pose.position.x = 0.0
    sp.pose.position.y = 0.0
    sp.pose.position.z = alt_m        # ENU: 正值 = 上

    # 在 orientation 裡帶 yaw → MAVROS 萃取後填入 traj.yaw（有限值）
    q = quaternion_from_euler(0.0, 0.0, yaw_rad)   # roll=0, pitch=0, yaw
    sp.pose.orientation.x = q[0]
    sp.pose.orientation.y = q[1]
    sp.pose.orientation.z = q[2]
    sp.pose.orientation.w = q[3]
    return sp


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
    """Set PX4 parameter via /mavros/param/set service."""
    try:
        rospy.wait_for_service('/mavros/param/set', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()
        req.param_id     = name
        req.value        = ParamValue(integer=0, real=float(value))
        for _ in range(retries):
            res = svc(req)
            if res.success:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"param_set({name}) failed: {e}")
    return False


def param_get(name):
    """Get PX4 parameter value (float) via /mavros/param/get service."""
    try:
        rospy.wait_for_service('/mavros/param/get', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        res = svc(ParamGetRequest(param_id=name))
        return res.value.real if res.success else None
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"param_get({name}) failed: {e}")
        return None


def set_mode(mode_str, retries=5):
    """Set PX4 flight mode string (e.g. 'OFFBOARD', 'AUTO.LAND')."""
    try:
        rospy.wait_for_service('/mavros/set_mode', timeout=5)
        svc = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        for _ in range(retries):
            res = svc(SetModeRequest(custom_mode=mode_str))
            if res.mode_sent:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"set_mode({mode_str}) failed: {e}")
    return False


def arm_vehicle(do_arm=True, retries=5):
    """Arm or disarm via /mavros/cmd/arming service."""
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        for _ in range(retries):
            res = svc(CommandBoolRequest(value=do_arm))
            if res.success:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"arming({do_arm}) failed: {e}")
    return False


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────

rospy.init_node('pfa_attitude_tracking', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

# Subscribers
rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

# Position setpoint publisher
sp_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# Wait for MAVROS FCU connection
rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected. FCU status={_vehicle_state.system_status}")
rospy.loginfo(f"  Task: alt={TARGET_ALT_M} m, pitch={TARGET_PITCH_DEG}°")
rospy.loginfo("  Syncing parameter cache from FCU...")
param_pull()

# ─────────────────────────────────────────────────────────────
# Step 0 – 設定 PFA_DES_PITCH / PFA_DES_ROLL 為 0°（起飛前歸零）
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Configuring PFA attitude target parameters...")

val = param_get('PFA_DES_PITCH')
rospy.loginfo(f"  PFA_DES_PITCH current = {val:.1f}°"
              if val is not None else "  PFA_DES_PITCH: read failed (firmware not rebuilt?)")

rospy.loginfo("  Setting PFA_DES_ROLL  = 0° ...")
rospy.loginfo("    " + ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  Setting PFA_DES_PITCH = 0° ...")
rospy.loginfo("    " + ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1 – 串流初始 setpoint（PX4 進入 OFFBOARD 前的必要條件）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming initial setpoints (5 s, z={TARGET_ALT_M} m ENU)...")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 5.0:
    sp_pub.publish(make_hover_setpoint())
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 2 – OFFBOARD + Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo("  Requesting OFFBOARD mode...")
set_mode('OFFBOARD')

t_wait = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'OFFBOARD':
    sp_pub.publish(make_hover_setpoint())
    if (rospy.Time.now() - t_wait).to_sec() > 5.0:
        rospy.logwarn("  WARNING: OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_vehicle_state.mode}")

rospy.loginfo("  Arming...")
arm_vehicle()

t_wait = rospy.Time.now()
while not rospy.is_shutdown() and not _vehicle_state.armed:
    sp_pub.publish(make_hover_setpoint())
    if (rospy.Time.now() - t_wait).to_sec() > 10.0:
        rospy.logerr("  ERROR: Failed to arm.")
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – 位置控制懸停至 1 m（pfa_pos_control, PFA_DES_PITCH=0°）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover at {TARGET_ALT_M} m  (PFA_DES_PITCH=0°, pfa_pos_control)")
rospy.loginfo(f"  Waiting for stable hover (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()

while not rospy.is_shutdown():
    sp_pub.publish(make_hover_setpoint())
    alt     = get_altitude()
    alt_err = TARGET_ALT_M - alt
    stable_since = (stable_since or rospy.Time.now()) if abs(alt_err) < ALT_TOL else None

    now = rospy.Time.now()
    if (now - last_log).to_sec() > 1.0:
        s = (now - stable_since).to_sec() if stable_since else 0.0
        rospy.loginfo(f"  alt={alt:.2f} m (err={alt_err:+.2f})  "
                      f"pitch={get_pitch_deg():.1f}°  stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
        last_log = now

    if stable_since and (now - stable_since).to_sec() >= HOVER_STABLE_TIME:
        rospy.loginfo(f"  Stable hover at {alt:.2f} m.")
        break
    if (now - t_phase).to_sec() > 25.0:
        rospy.logwarn(f"  Hover timeout – alt={alt:.2f} m, continuing.")
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 4 – 水平懸停保持 HOVER_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
t_end    = rospy.Time.now() + rospy.Duration(HOVER_PHASE_DUR)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    sp_pub.publish(make_hover_setpoint())
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        rospy.loginfo(f"  alt={get_altitude():.2f} m  pitch={get_pitch_deg():.1f}°  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 5 – 漸進式 pitch ramp：透過 /mavros/param/set 逐步更新 PFA_DES_PITCH
#
#  Python → /mavros/param/set PFA_DES_PITCH = θ(t)
#                ↓ parameters_update()
#  pfa_pos_control:
#    attitude_des = (PFA_DES_ROLL, PFA_DES_PITCH, safe_yaw)   ← 讀參數
#    thrust_body  = rotateVectorInverse(Kp*err_pos + ... , q_current)  ← PD 計算
#    → vehicle_attitude_setpoint
#                ↓
#  pfa_att_control → 驅動機體到 pitch=45°
# ─────────────────────────────────────────────────────────────
# abs() 確保正負 pitch 都能正確計算等待時間
PARAM_STEP_WAIT = PITCH_RAMP_TIME / (abs(TARGET_PITCH_DEG) / PARAM_STEP_DEG)

# 物理限制：_thrust_xy_max = 0.3，飽和角 = arcsin(0.3 / hover_thrust) ≈ ±31.6°
_sat_limit = math.degrees(math.asin(0.3 / ((1.4 * 9.81) / 24.0)))
rospy.loginfo(f"\n[Step 5] Pitch ramp: 0° → {TARGET_PITCH_DEG:.0f}° "
              f"(step={PARAM_STEP_DEG:.0f}°, interval={PARAM_STEP_WAIT:.1f} s/step)")
rospy.loginfo(f"  XY thrust saturation limit: ±{_sat_limit:.1f}°  "
              f"({'OK' if abs(TARGET_PITCH_DEG) <= _sat_limit else 'WARNING: near/over saturation'})")
rospy.loginfo("  → /mavros/param/set PFA_DES_PITCH")
rospy.loginfo("  → /mavros/setpoint_position/local 持續送位置目標讓 pfa_pos_control 計算 thrust")

current_pitch_cmd = 0.0
# 正負 pitch 均支援：step 方向跟隨目標符號，clamp 防止 overshoot
while not rospy.is_shutdown() and abs(current_pitch_cmd - TARGET_PITCH_DEG) > 0.01:
    step       = math.copysign(PARAM_STEP_DEG, TARGET_PITCH_DEG - current_pitch_cmd)
    next_pitch = current_pitch_cmd + step
    if (step > 0 and next_pitch > TARGET_PITCH_DEG) or \
       (step < 0 and next_pitch < TARGET_PITCH_DEG):
        next_pitch = TARGET_PITCH_DEG

    ok = param_set('PFA_DES_PITCH', next_pitch)
    rospy.loginfo(f"  PFA_DES_PITCH = {next_pitch:.1f}° ... {'OK' if ok else 'FAILED'}")
    current_pitch_cmd = next_pitch

    t_step   = rospy.Time.now() + rospy.Duration(PARAM_STEP_WAIT)
    last_log = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < t_step:
        sp_pub.publish(make_hover_setpoint())
        now = rospy.Time.now()
        if (now - last_log).to_sec() > 0.5:
            rospy.loginfo(f"    alt={get_altitude():.2f} m  "
                          f"pitch_cmd={current_pitch_cmd:.1f}°  "
                          f"pitch_now={get_pitch_deg():.1f}°")
            last_log = now
        rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 6 – 維持 pitch=45°, alt=1 m，持續追蹤 TRACK_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Holding pitch={TARGET_PITCH_DEG:.0f}°, "
              f"alt={TARGET_ALT_M} m for {TRACK_PHASE_DUR:.0f} s...")
rospy.loginfo(f"  {'Time':>6}  {'Alt':>6}  {'AltErr':>7}  {'PitchCmd':>9}  {'PitchNow':>9}")
rospy.loginfo("  " + "─" * 46)

t_track  = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown() and (rospy.Time.now() - t_track).to_sec() < TRACK_PHASE_DUR:
    sp_pub.publish(make_hover_setpoint())
    now     = rospy.Time.now()
    elapsed = (now - t_track).to_sec()
    alt     = get_altitude()

    if (now - last_log).to_sec() >= 0.5:
        rospy.loginfo(f"  {elapsed:6.1f}s  {alt:6.2f}m  {TARGET_ALT_M - alt:+7.2f}m  "
                      f"{TARGET_PITCH_DEG:8.1f}°  {get_pitch_deg():8.1f}°")
        last_log = now
    rate.sleep()

rospy.loginfo("  Attitude tracking complete.")

# ─────────────────────────────────────────────────────────────
# Step 7 – 降落前先將 PFA_DES_PITCH 歸零
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 7] Resetting PFA_DES_PITCH to 0° before landing...")
param_set('PFA_DES_PITCH', 0.0)
rospy.sleep(1.0)

rospy.loginfo("Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
