#!/usr/bin/env python3
"""
PFA Yaw Tracking Task (MAVROS / ROS1 Noetic)
=============================================
MAVROS 版本，對應 pfa_yaw_tracking.py (pymavlink 版)

Task: 懸停於高度 1 m，yaw 從初始航向平滑旋轉至目標角度

控制架構：
  Python → /mavros/setpoint_position/local (PoseStamped, ENU,
            z=+1 m, orientation.yaw = θ(t))
  MAVROS → SET_POSITION_TARGET_LOCAL_NED (position + yaw)
  PX4:  trajectory_setpoint.yaw = θ(t)  (有限值，無 NaN)
        pfa_pos_control: safe_yaw = traj.yaw
          attitude_des = (PFA_DES_ROLL, PFA_DES_PITCH, safe_yaw)
        pfa_att_control → 驅動機體到 yaw = TARGET_YAW_DEG

Yaw 約定 (ENU, MAVROS)：
  0° = 朝東 (East)
  +90° = 朝北 (North)
  -90° = 朝南 (South)
  ±180° = 朝西 (West)
  正值 = 逆時針旋轉 (viewed from above)

注意：yaw 透過 PoseStamped orientation 四元數傳遞，MAVROS 自動轉換為
      SET_POSITION_TARGET_LOCAL_NED.yaw（有限值，防止 _checkAllFinite NaN）。
      ramp 為連續線性（非離散 PARAM_SET），不需要 PFA_DES_YAW 參數。

執行方式：
  rosrun <your_package> pfa_yaw_tracking_mavros.py
  或
  python3 pfa_yaw_tracking_mavros.py  (ROS 環境已 source)
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
    ParamSet,    ParamSetRequest,
    ParamPull,   ParamPullRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M     = 1.0       # 懸停高度 (m，ENU: z+ 方向)
TARGET_YAW_DEG   = 90.0      # 目標 yaw (deg, ENU: +90=North, 逆時針)
TARGET_ROLL_DEG  = 0.0       # roll/pitch 維持 0°
TARGET_PITCH_DEG = 0.0

HOVER_PHASE_DUR  = 8.0       # 水平懸停穩定後等待時間 (s)
YAW_RAMP_TIME    = 4.0       # yaw ramp 持續時間 (s)
TRACK_PHASE_DUR  = 30.0      # 維持目標 yaw 的追蹤時長 (s)

ALT_TOL          = 0.15      # 高度容忍範圍 (m)
HOVER_STABLE_TIME = 3.0      # 在容忍範圍內連續 N 秒才確認穩定

CTRL_HZ          = 20        # 控制迴路頻率 (Hz)

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


def get_yaw_rad():
    """從 local_position/pose 取得當前 yaw (rad，ENU 約定)。"""
    o = _local_pose.pose.orientation
    _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return yaw


def get_yaw_deg():
    """從 local_position/pose 取得當前 yaw (deg，ENU 約定)。"""
    return math.degrees(get_yaw_rad())


def angle_wrap(a):
    """Wrap angle to [-π, π] (shortest path)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


# ─────────────────────────────────────────────────────────────
# Setpoint helper
# ─────────────────────────────────────────────────────────────

def make_hover_setpoint(alt_m=None, yaw_rad=None):
    """
    ENU 座標系的位置 setpoint (PoseStamped)。
    yaw 透過 orientation 四元數傳遞 → MAVROS 萃取後填入 traj.yaw（有限值），
    不觸發 pfa_att_control 的 _checkAllFinite 零化問題。
    """
    if alt_m is None:
        alt_m = TARGET_ALT_M
    if yaw_rad is None:
        yaw_rad = get_yaw_rad()

    sp = PoseStamped()
    sp.header.stamp    = rospy.Time.now()
    sp.header.frame_id = 'map'

    sp.pose.position.x = 0.0
    sp.pose.position.y = 0.0
    sp.pose.position.z = alt_m

    q = quaternion_from_euler(0.0, 0.0, yaw_rad)
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
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        for _ in range(retries):
            res = svc(req)
            if res.success:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"param_set({name}) failed: {e}")
    return False


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

rospy.init_node('pfa_yaw_tracking', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

# Subscribers
rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

# Position setpoint publisher
sp_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# Wait for MAVROS FCU connection
rospy.loginfo("Waiting for MAVROS FCU connection and ready state (status >= 3)...")
while not rospy.is_shutdown() and not (
        _vehicle_state.connected and _vehicle_state.system_status >= 3):
    rate.sleep()
rospy.loginfo(f"  Connected. FCU status={_vehicle_state.system_status}")
rospy.loginfo(f"  Task: alt={TARGET_ALT_M} m, yaw={TARGET_YAW_DEG}° (ENU)")
rospy.loginfo("  Syncing parameter cache from FCU...")
param_pull()

# ─────────────────────────────────────────────────────────────
# Step 0 – 起飛前確保 roll/pitch 參數歸零（yaw 不需要參數）
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Configuring PFA attitude target parameters...")
rospy.loginfo("  Setting PFA_DES_ROLL  = 0° ... " +
              ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  Setting PFA_DES_PITCH = 0° ... " +
              ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1 – 串流初始 setpoint（PX4 進入 OFFBOARD 前的必要條件）
# ─────────────────────────────────────────────────────────────
INIT_YAW_RAD = 0.0   # 起飛時送 yaw=0°（ENU=朝東），確保有限值

rospy.loginfo(f"\n[Step 1] Streaming initial setpoints (5 s, z={TARGET_ALT_M} m, yaw=0°)...")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 5.0:
    sp_pub.publish(make_hover_setpoint(yaw_rad=INIT_YAW_RAD))
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 2 – OFFBOARD + Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo("  Requesting OFFBOARD mode...")
set_mode('OFFBOARD')

t_wait = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'OFFBOARD':
    sp_pub.publish(make_hover_setpoint(yaw_rad=INIT_YAW_RAD))
    if (rospy.Time.now() - t_wait).to_sec() > 5.0:
        rospy.logwarn("  WARNING: OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_vehicle_state.mode}")

rospy.loginfo("  Arming...")
arm_vehicle()

t_wait = rospy.Time.now()
while not rospy.is_shutdown() and not _vehicle_state.armed:
    sp_pub.publish(make_hover_setpoint(yaw_rad=INIT_YAW_RAD))
    if (rospy.Time.now() - t_wait).to_sec() > 10.0:
        rospy.logerr("  ERROR: Failed to arm.")
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – 位置控制懸停至 1 m (yaw=0°)
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover at {TARGET_ALT_M} m  (yaw=0°, PFA_DES_ROLL/PITCH=0°)")
rospy.loginfo(f"  Waiting for stable hover (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()

while not rospy.is_shutdown():
    sp_pub.publish(make_hover_setpoint(yaw_rad=INIT_YAW_RAD))
    alt     = get_altitude()
    alt_err = TARGET_ALT_M - alt
    stable_since = (stable_since or rospy.Time.now()) if abs(alt_err) < ALT_TOL else None

    now = rospy.Time.now()
    if (now - last_log).to_sec() > 1.0:
        s = (now - stable_since).to_sec() if stable_since else 0.0
        rospy.loginfo(f"  alt={alt:.2f} m (err={alt_err:+.2f})  "
                      f"yaw={get_yaw_deg():.1f}°  stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
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
    sp_pub.publish(make_hover_setpoint(yaw_rad=INIT_YAW_RAD))
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        rospy.loginfo(f"  alt={get_altitude():.2f} m  yaw={get_yaw_deg():.1f}°  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 5 – Yaw ramp：連續線性旋轉至 TARGET_YAW_DEG (ENU)
#
#  yaw 透過 PoseStamped orientation 四元數傳遞，
#  MAVROS 萃取後填入 trajectory_setpoint.yaw（有限值）；
#  pfa_pos_control 讀取 → safe_yaw → attitude_des.z。
#  無需 PARAM_SET，每個控制週期更新一次（20 Hz）。
# ─────────────────────────────────────────────────────────────
initial_yaw_rad = get_yaw_rad()
target_yaw_rad  = math.radians(TARGET_YAW_DEG)
delta_yaw_rad   = angle_wrap(target_yaw_rad - initial_yaw_rad)

rospy.loginfo(f"\n[Step 5] Yaw ramp: {math.degrees(initial_yaw_rad):.1f}° → {TARGET_YAW_DEG:.1f}°  "
              f"(delta={math.degrees(delta_yaw_rad):+.1f}°, duration={YAW_RAMP_TIME:.1f} s)")
rospy.loginfo("  → yaw 內嵌於 PoseStamped orientation quaternion（無 PARAM_SET）")

t_ramp_start = rospy.Time.now()
last_log     = rospy.Time.now()

while not rospy.is_shutdown():
    elapsed  = (rospy.Time.now() - t_ramp_start).to_sec()
    frac     = min(elapsed / YAW_RAMP_TIME, 1.0)
    cmd_yaw  = initial_yaw_rad + frac * delta_yaw_rad

    sp_pub.publish(make_hover_setpoint(yaw_rad=cmd_yaw))

    now = rospy.Time.now()
    if (now - last_log).to_sec() > 0.5:
        rospy.loginfo(f"  alt={get_altitude():.2f} m  "
                      f"yaw_cmd={math.degrees(cmd_yaw):.1f}°  "
                      f"yaw_now={get_yaw_deg():.1f}°")
        last_log = now

    if frac >= 1.0:
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 6 – 維持 yaw=TARGET_YAW_DEG, alt=1 m，持續追蹤 TRACK_PHASE_DUR 秒
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Holding yaw={TARGET_YAW_DEG:.1f}°, "
              f"alt={TARGET_ALT_M} m for {TRACK_PHASE_DUR:.0f} s...")
rospy.loginfo(f"  {'Time':>6}  {'Alt':>6}  {'AltErr':>7}  {'YawCmd':>8}  {'YawNow':>8}")
rospy.loginfo("  " + "─" * 46)

t_track  = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown() and (rospy.Time.now() - t_track).to_sec() < TRACK_PHASE_DUR:
    sp_pub.publish(make_hover_setpoint(yaw_rad=target_yaw_rad))
    now     = rospy.Time.now()
    elapsed = (now - t_track).to_sec()
    alt     = get_altitude()

    if (now - last_log).to_sec() >= 0.5:
        rospy.loginfo(f"  {elapsed:6.1f}s  {alt:6.2f}m  {TARGET_ALT_M - alt:+7.2f}m  "
                      f"{TARGET_YAW_DEG:7.1f}°  {get_yaw_deg():7.1f}°")
        last_log = now
    rate.sleep()

rospy.loginfo("  Yaw tracking complete.")

# ─────────────────────────────────────────────────────────────
# Step 7 – 降落（yaw 不需要歸零參數，直接 AUTO.LAND）
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 7] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
