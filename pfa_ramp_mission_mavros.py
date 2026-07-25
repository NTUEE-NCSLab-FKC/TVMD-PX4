#!/usr/bin/env python3
"""
PFA Ramp Mission (MAVROS / ROS 1 Noetic)
==========================================
Task: 往前爬坡起飛 → 巡航 → 斜坡前進下降 → AUTO.LAND

軌跡 (ENU，yaw=0° 朝東)：

  Phase 1 – 爬坡起飛
    x(t) = x₀ + t × PATH_SPEED_MPS
    z(t) = z₀ + t × CLIMB_SPEED_MPS          (至 TARGET_ALT_M 止)

  Phase 2 – 水平巡航
    x(t) = x_c + t × PATH_SPEED_MPS
    z    = TARGET_ALT_M
    持續 CRUISE_DIST_M ÷ PATH_SPEED_MPS 秒

  Phase 3 – 斜坡前進下降
    x(t) = x_d + t × PATH_SPEED_MPS
    z(t) = TARGET_ALT_M - t × DESCENT_SPEED_MPS  (至 LAND_ALT_M 止)

  Phase 4 – AUTO.LAND

PFA_DES_ROLL = PFA_DES_PITCH = 0° 全程；位置控制器自動產生
所需推力，跟隨斜坡位置指令。

執行方式：
  python3 pfa_ramp_mission_mavros.py  (ROS 環境已 source)
"""

import sys
import math
import threading

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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
TARGET_ALT_M          = 30.0  # 巡航高度 (m)
CRUISE_DIST_M         = 100.0 # 水平巡航距離 (m)
PATH_SPEED_MPS        = 2.0   # 水平前進速度 (m/s, ENU x+)
CLIMB_SPEED_MPS       = 1.0   # 爬升速率 (m/s)
DESCENT_SPEED_MPS     = 1.0   # 下降速率 (m/s)

TAKEOFF_STREAM_ALT_M  = 2.0   # 串流 setpoint 起始高度，供 OFFBOARD 前置條件使用
LIFTOFF_CONFIRM_ALT_M = 1.5   # 確認已離地的高度門檻
LAND_ALT_M            = 3.0   # 切換 AUTO.LAND 的高度門檻

CTRL_HZ               = 20    # setpoint 發布頻率

# ─────────────────────────────────────────────────────────────
# Derived (mission summary)
# ─────────────────────────────────────────────────────────────
_CLIMB_DUR_S   = (TARGET_ALT_M - TAKEOFF_STREAM_ALT_M) / CLIMB_SPEED_MPS
_CLIMB_DIST_M  = PATH_SPEED_MPS * _CLIMB_DUR_S
_CRUISE_DUR_S  = CRUISE_DIST_M / PATH_SPEED_MPS
_DESCENT_DUR_S = (TARGET_ALT_M - LAND_ALT_M) / DESCENT_SPEED_MPS
_DESCENT_DIST_M = PATH_SPEED_MPS * _DESCENT_DUR_S
_TOTAL_DIST_M  = _CLIMB_DIST_M + CRUISE_DIST_M + _DESCENT_DIST_M
_TOTAL_DUR_S   = _CLIMB_DUR_S + _CRUISE_DUR_S + _DESCENT_DUR_S

# ─────────────────────────────────────────────────────────────
# Shared setpoint state (Timer publishes; main thread updates)
# ─────────────────────────────────────────────────────────────
_sp      = {'x': 0.0, 'y': 0.0, 'z': TAKEOFF_STREAM_ALT_M, 'yaw': 0.0}
_sp_lock = threading.Lock()


def update_sp(x=None, y=None, z=None, yaw=None):
    with _sp_lock:
        if x   is not None: _sp['x']   = x
        if y   is not None: _sp['y']   = y
        if z   is not None: _sp['z']   = z
        if yaw is not None: _sp['yaw'] = yaw


def _make_sp_msg():
    with _sp_lock:
        x, y, z, yaw_rad = _sp['x'], _sp['y'], _sp['z'], _sp['yaw']
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
# Global vehicle state
# ─────────────────────────────────────────────────────────────
_vehicle_state = State()
_local_pose    = PoseStamped()


def _cb_state(msg): global _vehicle_state; _vehicle_state = msg
def _cb_pose(msg):  global _local_pose;    _local_pose    = msg


def get_altitude(): return _local_pose.pose.position.z
def get_x():        return _local_pose.pose.position.x
def get_y():        return _local_pose.pose.position.y


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
# ROS init + 20 Hz Timer
# ─────────────────────────────────────────────────────────────
rospy.init_node('pfa_ramp_mission', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)

sp_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

_sp_timer = rospy.Timer(
    rospy.Duration(1.0 / CTRL_HZ),
    lambda e: sp_pub.publish(_make_sp_msg()))

rospy.loginfo("Waiting for MAVROS FCU connection and ready state (status >= 3)...")
while not rospy.is_shutdown() and not (
        _vehicle_state.connected and _vehicle_state.system_status >= 3):
    rate.sleep()
rospy.loginfo(f"  Connected. FCU status={_vehicle_state.system_status}  mode={_vehicle_state.mode}")
rospy.loginfo(f"  Mission plan:")
rospy.loginfo(f"    Cruise alt    : {TARGET_ALT_M:.0f} m")
rospy.loginfo(f"    Cruise dist   : {CRUISE_DIST_M:.0f} m  ({_CRUISE_DUR_S:.0f} s)")
rospy.loginfo(f"    Climb  slope  : {PATH_SPEED_MPS:.1f} m/s horiz + {CLIMB_SPEED_MPS:.1f} m/s vert  "
              f"≈ {_CLIMB_DIST_M:.0f} m horiz / {_CLIMB_DUR_S:.0f} s")
rospy.loginfo(f"    Descent slope : {PATH_SPEED_MPS:.1f} m/s horiz + {DESCENT_SPEED_MPS:.1f} m/s vert  "
              f"≈ {_DESCENT_DIST_M:.0f} m horiz / {_DESCENT_DUR_S:.0f} s")
rospy.loginfo(f"    Total horiz   : ≈ {_TOTAL_DIST_M:.0f} m  ({_TOTAL_DUR_S:.0f} s)")
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

# ─────────────────────────────────────────────────────────────
# Step 1 – Stream setpoints (OFFBOARD precondition)
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming initial setpoints "
              f"(5 s, z={TAKEOFF_STREAM_ALT_M:.0f} m, yaw=0°)...")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 5.0:
    rate.sleep()   # Timer is already publishing

# ─────────────────────────────────────────────────────────────
# Step 2 – OFFBOARD + Arm
# ─────────────────────────────────────────────────────────────
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
        _sp_timer.shutdown()
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – Wait for liftoff confirmation
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Waiting for liftoff (alt > {LIFTOFF_CONFIRM_ALT_M:.1f} m)...")
last_log = rospy.Time.now()
t_phase  = rospy.Time.now()

while not rospy.is_shutdown():
    alt = get_altitude()
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 1.0:
        rospy.loginfo(f"  alt={alt:.2f} m")
        last_log = now
    if alt >= LIFTOFF_CONFIRM_ALT_M:
        rospy.loginfo(f"  Liftoff confirmed at alt={alt:.2f} m.")
        break
    if (now - t_phase).to_sec() > 20.0:
        rospy.logwarn("  Liftoff timeout – continuing anyway.")
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 4 – Phase 1: Climb ramp (forward + ascent)
# ─────────────────────────────────────────────────────────────
x0_climb = get_x()
z0_climb = get_altitude()

rospy.loginfo(f"\n[Step 4] CLIMB RAMP: start ({x0_climb:.1f}, {z0_climb:.1f} m) "
              f"→ z={TARGET_ALT_M:.0f} m")
rospy.loginfo(f"  Speed: horiz {PATH_SPEED_MPS:.1f} m/s + vert {CLIMB_SPEED_MPS:.1f} m/s")
rospy.loginfo(f"  {'Time':>6}  {'Xcmd':>6}  {'Xnow':>6}  {'Zcmd':>6}  {'Znow':>6}")
rospy.loginfo("  " + "─" * 40)

t_phase  = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_phase).to_sec()

    x_cmd = x0_climb + elapsed * PATH_SPEED_MPS
    z_cmd = z0_climb + elapsed * CLIMB_SPEED_MPS

    if z_cmd >= TARGET_ALT_M:
        z_cmd = TARGET_ALT_M
        update_sp(x=x_cmd, y=0.0, z=z_cmd)
        if (now - last_log).to_sec() > 1.0:
            rospy.loginfo(f"  {elapsed:6.1f}s  {x_cmd:6.1f}m  {get_x():6.1f}m  "
                          f"{z_cmd:6.1f}m  {get_altitude():6.1f}m")
        break

    update_sp(x=x_cmd, y=0.0, z=z_cmd)

    if (now - last_log).to_sec() > 1.0:
        rospy.loginfo(f"  {elapsed:6.1f}s  {x_cmd:6.1f}m  {get_x():6.1f}m  "
                      f"{z_cmd:6.1f}m  {get_altitude():6.1f}m")
        last_log = now
    rate.sleep()

x_cruise_start = x_cmd
rospy.loginfo(f"\n  Climb complete. x ≈ {x_cruise_start:.1f} m  z_cmd = {TARGET_ALT_M:.0f} m")

# ─────────────────────────────────────────────────────────────
# Step 5 – Phase 2: Level cruise
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] CRUISE: z={TARGET_ALT_M:.0f} m  →  {CRUISE_DIST_M:.0f} m forward "
              f"({_CRUISE_DUR_S:.0f} s)")
rospy.loginfo(f"  {'Time':>6}  {'Xcmd':>7}  {'Xnow':>7}  {'Znow':>6}  {'Remaining':>10}")
rospy.loginfo("  " + "─" * 46)

t_phase  = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_phase).to_sec()

    x_cmd = x_cruise_start + elapsed * PATH_SPEED_MPS
    update_sp(x=x_cmd, y=0.0, z=TARGET_ALT_M)

    if (now - last_log).to_sec() > 1.0:
        remaining = CRUISE_DIST_M - (x_cmd - x_cruise_start)
        rospy.loginfo(f"  {elapsed:6.1f}s  {x_cmd:7.1f}m  {get_x():7.1f}m  "
                      f"{get_altitude():6.1f}m  {max(0.0, remaining):8.1f}m")
        last_log = now

    if x_cmd - x_cruise_start >= CRUISE_DIST_M:
        break
    rate.sleep()

x_descent_start = x_cmd
rospy.loginfo(f"\n  Cruise complete. x ≈ {x_descent_start:.1f} m")

# ─────────────────────────────────────────────────────────────
# Step 6 – Phase 3: Descent ramp (forward + descent)
# ─────────────────────────────────────────────────────────────
x0_desc = get_x()

rospy.loginfo(f"\n[Step 6] DESCENT RAMP: z={TARGET_ALT_M:.0f} m → {LAND_ALT_M:.0f} m "
              f"while flying forward")
rospy.loginfo(f"  Speed: horiz {PATH_SPEED_MPS:.1f} m/s + vert {DESCENT_SPEED_MPS:.1f} m/s")
rospy.loginfo(f"  {'Time':>6}  {'Xcmd':>7}  {'Xnow':>7}  {'Zcmd':>6}  {'Znow':>6}")
rospy.loginfo("  " + "─" * 40)

t_phase  = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_phase).to_sec()

    x_cmd = x0_desc + elapsed * PATH_SPEED_MPS
    z_cmd = TARGET_ALT_M - elapsed * DESCENT_SPEED_MPS

    if z_cmd <= LAND_ALT_M:
        z_cmd = LAND_ALT_M
        update_sp(x=x_cmd, y=0.0, z=z_cmd)
        if (now - last_log).to_sec() > 0.5:
            rospy.loginfo(f"  {elapsed:6.1f}s  {x_cmd:7.1f}m  {get_x():7.1f}m  "
                          f"{z_cmd:6.1f}m  {get_altitude():6.1f}m")
        break

    update_sp(x=x_cmd, y=0.0, z=z_cmd)

    if (now - last_log).to_sec() > 1.0:
        rospy.loginfo(f"  {elapsed:6.1f}s  {x_cmd:7.1f}m  {get_x():7.1f}m  "
                      f"{z_cmd:6.1f}m  {get_altitude():6.1f}m")
        last_log = now
    rate.sleep()

rospy.loginfo(f"\n  Descent ramp complete. x ≈ {get_x():.1f} m  alt ≈ {get_altitude():.1f} m")

# ─────────────────────────────────────────────────────────────
# Step 7 – Land
# ─────────────────────────────────────────────────────────────
_sp_timer.shutdown()
rospy.loginfo("\n[Step 7] Switching to AUTO.LAND...")
set_mode('AUTO.LAND')
rospy.loginfo("  Waiting for touchdown (20 s)...")
rospy.sleep(20.0)
rospy.loginfo("Done.")
