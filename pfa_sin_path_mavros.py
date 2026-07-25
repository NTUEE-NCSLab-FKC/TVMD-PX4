#!/usr/bin/env python3
"""
PFA Sin Wave Path Tracking (MAVROS / ROS1 Noetic)
===================================================
MAVROS 版本，對應 pfa_sin_path.py (pymavlink 版)

Task: 追蹤水平正弦波路徑，roll 與 pitch 維持 0 度

軌跡 (ENU，MAVROS，俯視圖)：
  x(t) = x₀ + t × PATH_SPEED_MPS                              (朝東前進)
  y(t) = y₀ + SIN_AMPLITUDE_M × sin(2π t / SIN_PERIOD_S)     (南北振盪)
  z    = TARGET_ALT_M                                           (高度保持)
  yaw  = 0°  (ENU 約定：朝東)

空間波長：λ = PATH_SPEED_MPS × SIN_PERIOD_S
峰值側向速度：Vy_max = SIN_AMPLITUDE_M × 2π / SIN_PERIOD_S

控制架構：
  Python → /mavros/setpoint_position/local (PoseStamped, ENU)
  MAVROS → SET_POSITION_TARGET_LOCAL_NED (自動 ENU→NED)
  PX4:  pfa_pos_control → thrust (追蹤 sin 波位置)
                        → attitude_des = (0, 0, safe_yaw)   ← PFA_DES_ROLL/PITCH=0
        pfa_att_control → roll=0°, pitch=0°

執行方式：
  python3 pfa_sin_path_mavros.py  (ROS 環境已 source)
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
TARGET_ALT_M      = 1.0    # 懸停高度 (m，ENU z+)
PATH_SPEED_MPS    = 0.5    # 前進速度 (m/s，ENU x+)
SIN_AMPLITUDE_M   = 1.0    # 正弦波幅度 (m，ENU y 方向，南北振盪)
SIN_PERIOD_S      = 4.0    # 振盪週期 (s/cycle)
NUM_CYCLES        = 4      # 完整週期數

HOVER_PHASE_DUR   = 8.0
ALT_TOL           = 0.15
HOVER_STABLE_TIME = 3.0

CTRL_HZ           = 20

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
TRACK_DUR_S      = NUM_CYCLES * SIN_PERIOD_S
TOTAL_PATH_M     = PATH_SPEED_MPS * TRACK_DUR_S
SPATIAL_LAMBDA_M = PATH_SPEED_MPS * SIN_PERIOD_S
PEAK_VY_MPS      = SIN_AMPLITUDE_M * 2.0 * math.pi / SIN_PERIOD_S

# ─────────────────────────────────────────────────────────────
# Global state (ROS subscribers)
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


def get_altitude():
    return _local_pose.pose.position.z


def get_xy():
    return _local_pose.pose.position.x, _local_pose.pose.position.y


def get_roll_pitch_deg():
    o = _imu_data.orientation
    roll, pitch, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(roll), math.degrees(pitch)


# ─────────────────────────────────────────────────────────────
# Setpoint helper
# ─────────────────────────────────────────────────────────────

def make_setpoint(x, y, z, yaw_rad=0.0):
    """ENU PoseStamped with explicit yaw (prevents NaN in trajectory_setpoint.yaw)."""
    sp = PoseStamped()
    sp.header.stamp    = rospy.Time.now()
    sp.header.frame_id = 'map'
    sp.pose.position.x = x
    sp.pose.position.y = y
    sp.pose.position.z = z
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
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('pfa_sin_path', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

sp_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected. FCU status={_vehicle_state.system_status}")
rospy.loginfo(f"  x: {PATH_SPEED_MPS} m/s × {TRACK_DUR_S:.0f} s = {TOTAL_PATH_M:.1f} m (ENU East)")
rospy.loginfo(f"  y: ±{SIN_AMPLITUDE_M:.1f} m sin wave, T={SIN_PERIOD_S:.0f} s, λ={SPATIAL_LAMBDA_M:.1f} m")
rospy.loginfo(f"  Vy_max = {PEAK_VY_MPS:.2f} m/s,  cycles = {NUM_CYCLES}")
rospy.loginfo(f"  Attitude: roll=0°, pitch=0° (PFA 位置/姿態解耦)")
rospy.loginfo("  Syncing parameter cache from FCU...")
param_pull()

# ─────────────────────────────────────────────────────────────
# Step 0 – 確保 PFA_DES_ROLL/PITCH = 0°（水平飛行）
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0° (level flight)...")
rospy.loginfo("  PFA_DES_ROLL  = 0° ... " +
              ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH = 0° ... " +
              ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1 – 串流初始 setpoint（OFFBOARD 前置條件）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming initial setpoints (5 s)...")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 5.0:
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_ALT_M))
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 2 – OFFBOARD + Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo("  Requesting OFFBOARD mode...")
set_mode('OFFBOARD')

t_wait = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'OFFBOARD':
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_ALT_M))
    if (rospy.Time.now() - t_wait).to_sec() > 5.0:
        rospy.logwarn("  WARNING: OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_vehicle_state.mode}")

rospy.loginfo("  Arming...")
arm_vehicle()

t_wait = rospy.Time.now()
while not rospy.is_shutdown() and not _vehicle_state.armed:
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_ALT_M))
    if (rospy.Time.now() - t_wait).to_sec() > 10.0:
        rospy.logerr("  ERROR: Failed to arm.")
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – 位置控制懸停至 1 m
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Stable hover at {TARGET_ALT_M} m...")
rospy.loginfo(f"  Waiting (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()

while not rospy.is_shutdown():
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_ALT_M))
    alt     = get_altitude()
    alt_err = TARGET_ALT_M - alt
    stable_since = (stable_since or rospy.Time.now()) if abs(alt_err) < ALT_TOL else None
    now = rospy.Time.now()
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
# Step 4 – Level hover hold
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
t_end    = rospy.Time.now() + rospy.Duration(HOVER_PHASE_DUR)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_ALT_M))
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        rospy.loginfo(f"  alt={get_altitude():.2f} m  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

# Record origin
x0, y0 = get_xy()
rospy.loginfo(f"  Path origin: ENU ({x0:.2f}, {y0:.2f}) m")

# ─────────────────────────────────────────────────────────────
# Step 5 – Sin wave path tracking
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Sin wave path: {NUM_CYCLES} cycles × {SIN_PERIOD_S:.0f} s = {TRACK_DUR_S:.0f} s")
rospy.loginfo(f"  {'Time':>6}  {'Xcmd':>6}  {'Xnow':>6}  {'Ycmd':>6}  {'Ynow':>6}  "
              f"{'Alt':>5}  {'Roll':>7}  {'Pitch':>7}")
rospy.loginfo("  " + "─" * 66)

t_track  = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_track).to_sec()
    if elapsed >= TRACK_DUR_S:
        elapsed = TRACK_DUR_S

    x_cmd = x0 + elapsed * PATH_SPEED_MPS
    y_cmd = y0 + SIN_AMPLITUDE_M * math.sin(
        2.0 * math.pi * elapsed / SIN_PERIOD_S)

    sp_pub.publish(make_setpoint(x_cmd, y_cmd, TARGET_ALT_M))

    if (now - last_log).to_sec() >= 0.5:
        xn, yn  = get_xy()
        alt     = get_altitude()
        r_deg, p_deg = get_roll_pitch_deg()
        rospy.loginfo(f"  {elapsed:6.1f}s  {x_cmd:6.2f}m  {xn:6.2f}m  "
                      f"{y_cmd:6.2f}m  {yn:6.2f}m  {alt:5.2f}m  "
                      f"{r_deg:6.1f}°  {p_deg:6.1f}°")
        last_log = now

    if elapsed >= TRACK_DUR_S:
        break
    rate.sleep()

rospy.loginfo("\n  Sin path complete.")

# ─────────────────────────────────────────────────────────────
# Step 6 – Hold end position for 10 s
# ─────────────────────────────────────────────────────────────
x_end, y_end = get_xy()
rospy.loginfo(f"\n[Step 6] Holding end position ({x_end:.2f}, {y_end:.2f}) m for 10 s...")
t_end    = rospy.Time.now() + rospy.Duration(10.0)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    sp_pub.publish(make_setpoint(x_end, y_end, TARGET_ALT_M))
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        r_deg, p_deg = get_roll_pitch_deg()
        xn, yn = get_xy()
        rospy.loginfo(f"  x={xn:.2f} y={yn:.2f} alt={get_altitude():.2f} m  "
                      f"roll={r_deg:.1f}° pitch={p_deg:.1f}°  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 7 – Land
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 7] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
