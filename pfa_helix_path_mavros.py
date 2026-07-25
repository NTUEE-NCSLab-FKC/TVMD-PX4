#!/usr/bin/env python3
"""
PFA Helix Path Tracking (MAVROS / ROS1 Noetic)
================================================
MAVROS 版本，對應 pfa_helix_path.py (pymavlink 版)

Task: 追蹤螺旋上升軌跡，roll=pitch=0°，yaw 跟隨切線方向

軌跡 (ENU，MAVROS，俯視為逆時針)：
  x(t) = x_c + R × cos(2π t / T_rev)    (ENU East  分量)
  y(t) = y_c + R × sin(2π t / T_rev)    (ENU North 分量)
  z(t) = h₀ + t × CLIMB_RATE_MPS        (ENU z+ = 上升)
  yaw(t) = atan2(cos θ, -sin θ)          (ENU yaw，面向前進方向)

切線速度  v_tan = 2π R / T_rev
向心加速度 a_c  = v_tan² / R

控制架構：
  Python → /mavros/setpoint_position/local (PoseStamped, ENU)
         → orientation quaternion 帶 yaw (防 NaN)
  MAVROS → SET_POSITION_TARGET_LOCAL_NED
  PX4:  pfa_pos_control → thrust (追蹤 3D 螺旋)
        pfa_att_control → roll=0°, pitch=0°, yaw 跟切線

執行方式：
  python3 pfa_helix_path_mavros.py  (ROS 環境已 source)
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
TARGET_START_ALT_M  = 1.0    # 螺旋起始高度 (m，ENU z+)
HELIX_RADIUS_M      = 1.5    # 螺旋半徑 (m)
CLIMB_RATE_MPS      = 0.3    # 垂直爬升速率 (m/s)
REV_PERIOD_S        = 8.0    # 每圈時間 (s/rev)
NUM_REVOLUTIONS     = 3      # 總圈數
YAW_TRACK_PATH      = True   # True: yaw 跟切線; False: yaw=0° 固定

HOVER_PHASE_DUR     = 8.0
APPROACH_DUR_S      = 4.0    # 從懸停中心移動到螺旋起點的時間 (s)
ALT_TOL             = 0.15
HOVER_STABLE_TIME   = 3.0

CTRL_HZ             = 20

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
TOTAL_DUR_S     = NUM_REVOLUTIONS * REV_PERIOD_S
TOTAL_CLIMB_M   = CLIMB_RATE_MPS * TOTAL_DUR_S
FINAL_ALT_M     = TARGET_START_ALT_M + TOTAL_CLIMB_M
V_TAN_MPS       = 2.0 * math.pi * HELIX_RADIUS_M / REV_PERIOD_S
A_CENTRIPETAL   = V_TAN_MPS ** 2 / HELIX_RADIUS_M
OMEGA           = 2.0 * math.pi / REV_PERIOD_S

# ─────────────────────────────────────────────────────────────
# Global state
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

def make_setpoint(x, y, z, yaw_rad=0.0):
    """ENU PoseStamped; orientation quaternion carries yaw (防 NaN)."""
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


def helix_yaw(theta):
    """ENU yaw aligned to helix tangent.
    d/dt[R*cos(θ), R*sin(θ)] = [-R*ω*sin(θ), +R*ω*cos(θ)]
    yaw_ENU = atan2(vy_ENU, vx_ENU) = atan2(cos θ, -sin θ)
    """
    return math.atan2(math.cos(theta), -math.sin(theta))


# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('pfa_helix_path', anonymous=False)
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
rospy.loginfo(f"  Helix: R={HELIX_RADIUS_M} m, T={REV_PERIOD_S} s/rev, "
              f"{NUM_REVOLUTIONS} revs, climb={CLIMB_RATE_MPS} m/s")
rospy.loginfo(f"  v_tan={V_TAN_MPS:.2f} m/s, a_centripetal={A_CENTRIPETAL:.3f} m/s²")
rospy.loginfo(f"  Alt: {TARGET_START_ALT_M:.1f} m → {FINAL_ALT_M:.1f} m "
              f"(+{TOTAL_CLIMB_M:.1f} m in {TOTAL_DUR_S:.0f} s)")
rospy.loginfo(f"  Attitude: roll=0°, pitch=0°, "
              f"{'yaw=切線方向' if YAW_TRACK_PATH else 'yaw=0° 固定'}")
rospy.loginfo("  Syncing parameter cache from FCU...")
param_pull()

# ─────────────────────────────────────────────────────────────
# Step 0 – PFA_DES_ROLL/PITCH = 0°
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0° (level flight)...")
rospy.loginfo("  PFA_DES_ROLL  = 0° ... " +
              ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH = 0° ... " +
              ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming initial setpoints (5 s)...")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 5.0:
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_START_ALT_M))
    rate.sleep()

rospy.loginfo("  Requesting OFFBOARD mode...")
set_mode('OFFBOARD')

t_wait = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'OFFBOARD':
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_START_ALT_M))
    if (rospy.Time.now() - t_wait).to_sec() > 5.0:
        rospy.logwarn("  WARNING: OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_vehicle_state.mode}")

rospy.loginfo("  Arming...")
arm_vehicle()

t_wait = rospy.Time.now()
while not rospy.is_shutdown() and not _vehicle_state.armed:
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_START_ALT_M))
    if (rospy.Time.now() - t_wait).to_sec() > 10.0:
        rospy.logerr("  ERROR: Failed to arm.")
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 2 – 穩定懸停至 TARGET_START_ALT_M
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 2] Stable hover at {TARGET_START_ALT_M} m...")
rospy.loginfo(f"  Waiting (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()

while not rospy.is_shutdown():
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_START_ALT_M))
    alt     = get_altitude()
    alt_err = TARGET_START_ALT_M - alt
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
# Step 3 – Level hover hold
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
t_end    = rospy.Time.now() + rospy.Duration(HOVER_PHASE_DUR)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_START_ALT_M))
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        rospy.loginfo(f"  alt={get_altitude():.2f} m  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

# Record helix center = current position (ENU)
x0, y0, _ = get_xyz()
hx = x0   # helix center x (ENU East)
hy = y0   # helix center y (ENU North)
rospy.loginfo(f"  Helix center: ENU ({hx:.2f}, {hy:.2f}) m")

# ─────────────────────────────────────────────────────────────
# Step 4 – 接近螺旋起點 (hx+R, hy)
# ─────────────────────────────────────────────────────────────
helix_start_yaw = helix_yaw(0.0)
rospy.loginfo(f"\n[Step 4] Approaching helix start "
              f"({hx+HELIX_RADIUS_M:.2f}, {hy:.2f}) m in {APPROACH_DUR_S:.0f} s...")

t_approach = rospy.Time.now()
last_log   = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_approach).to_sec()
    frac    = min(elapsed / APPROACH_DUR_S, 1.0)

    # Smooth cubic ease-in-out
    frac_smooth = frac * frac * (3.0 - 2.0 * frac)
    x_cmd  = hx + frac_smooth * HELIX_RADIUS_M
    y_cmd  = hy
    yaw_cmd = helix_start_yaw if YAW_TRACK_PATH else 0.0

    sp_pub.publish(make_setpoint(x_cmd, y_cmd, TARGET_START_ALT_M, yaw_rad=yaw_cmd))

    if (now - last_log).to_sec() > 1.0:
        xn, _, _ = get_xyz()
        rospy.loginfo(f"  x={xn:.2f} m (cmd={x_cmd:.2f})  "
                      f"alt={get_altitude():.2f} m  frac={frac*100:.0f}%")
        last_log = now

    if frac >= 1.0:
        break
    rate.sleep()

rospy.loginfo("  At helix start.")

# ─────────────────────────────────────────────────────────────
# Step 5 – 螺旋上升
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Helix ascent: {NUM_REVOLUTIONS} revs × {REV_PERIOD_S:.0f} s = "
              f"{TOTAL_DUR_S:.0f} s, {TARGET_START_ALT_M:.1f}→{FINAL_ALT_M:.1f} m")
rospy.loginfo(f"  {'Time':>6}  {'θ(°)':>6}  {'Xcmd':>6}  {'Ycmd':>6}  "
              f"{'AltCmd':>7}  {'AltNow':>7}  {'YawCmd':>8}  {'Roll':>6}  {'Pitch':>6}")
rospy.loginfo("  " + "─" * 76)

t_track  = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_track).to_sec()
    if elapsed >= TOTAL_DUR_S:
        elapsed = TOTAL_DUR_S

    theta   = OMEGA * elapsed
    x_cmd   = hx + HELIX_RADIUS_M * math.cos(theta)
    y_cmd   = hy + HELIX_RADIUS_M * math.sin(theta)
    z_cmd   = TARGET_START_ALT_M + elapsed * CLIMB_RATE_MPS   # ENU z+
    yaw_cmd = helix_yaw(theta) if YAW_TRACK_PATH else 0.0

    sp_pub.publish(make_setpoint(x_cmd, y_cmd, z_cmd, yaw_rad=yaw_cmd))

    if (now - last_log).to_sec() >= 0.5:
        alt_now        = get_altitude()
        r_deg, p_deg, yaw_fb = get_roll_pitch_yaw_deg()
        rospy.loginfo(f"  {elapsed:6.1f}s  {math.degrees(theta)%360:6.1f}°  "
                      f"{x_cmd:6.2f}m  {y_cmd:6.2f}m  "
                      f"{z_cmd:7.2f}m  {alt_now:7.2f}m  "
                      f"{math.degrees(yaw_cmd):7.1f}°  {r_deg:5.1f}°  {p_deg:5.1f}°")
        last_log = now

    if elapsed >= TOTAL_DUR_S:
        break
    rate.sleep()

rospy.loginfo("\n  Helix ascent complete.")

# ─────────────────────────────────────────────────────────────
# Step 6 – Hold end position for 10 s
# ─────────────────────────────────────────────────────────────
x_hold, y_hold, z_hold = get_xyz()
final_yaw = helix_yaw(OMEGA * TOTAL_DUR_S) if YAW_TRACK_PATH else 0.0

rospy.loginfo(f"\n[Step 6] Holding end position ({x_hold:.2f}, {y_hold:.2f}, "
              f"alt={z_hold:.2f} m) for 10 s...")
t_end    = rospy.Time.now() + rospy.Duration(10.0)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    sp_pub.publish(make_setpoint(x_hold, y_hold, z_hold, yaw_rad=final_yaw))
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        r_deg, p_deg, _ = get_roll_pitch_yaw_deg()
        rospy.loginfo(f"  alt={get_altitude():.2f} m  "
                      f"roll={r_deg:.1f}°  pitch={p_deg:.1f}°  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 7 – Land
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 7] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(20.0)
rospy.loginfo("Done.")
