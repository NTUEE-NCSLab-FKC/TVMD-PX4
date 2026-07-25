#!/usr/bin/env python3
"""
PFA Roll Forward (MAVROS / ROS 1 Noetic)
==========================================
MAVROS 版本，對應 pfa_roll_forward.py (pymavlink 版) 與 pfa_roll_forward_ros2.py (ROS 2 版)

Task: 直線前進，roll 從 +45° 到 -45° 來回振盪（正弦波）

軌跡 (ENU，MAVROS，yaw=0° 朝東)：
  x(t) = x₀ + t × PATH_SPEED_MPS   (朝東前進)
  y    = 0                           (側向不動)
  z    = TARGET_ALT_M                (高度保持)
  yaw  = 0°

PFA_DES_ROLL(t) = ROLL_AMP × sin(2π t / ROLL_PERIOD_S)
  t=0    →  0°     t=T/4 → +45°    t=T/2 →  0°
  t=3T/4 → -45°    t=T   →  0°   (一圈完成)

控制架構：
  rospy.Timer (20 Hz) → /mavros/setpoint_position/local (PoseStamped, ENU)
  主執行緒: /mavros/param/set PFA_DES_ROLL = ROLL_AMP × sin(ωt)  (~5 Hz)

與 pymavlink 版的關鍵差異：
  - ENU 座標系 (x=東, y=北, z=上)；yaw=0° 朝東
  - rospy.Timer 在獨立執行緒以 20 Hz 發佈 setpoint，主執行緒可安全阻塞在 param service
  - /mavros/param/set 服務呼叫 (~5 Hz) 取代 pymavlink param_set_async (fire-and-forget)

執行方式：
  python3 pfa_roll_forward_mavros.py  (ROS 環境已 source)
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
TARGET_ALT_M    = 1.0    # 懸停高度 (m，ENU z+)
PATH_SPEED_MPS  = 0.5    # 前進速度 (m/s，ENU x+)
ROLL_AMP_DEG    = 45.0   # roll 振盪幅度 (deg)；飽和限制 ≈ ±31.6°
ROLL_PERIOD_S   = 6.0    # 振盪週期 (s)
NUM_CYCLES      = 3      # 週期數

HOVER_PHASE_DUR   = 8.0
ALT_TOL           = 0.15
HOVER_STABLE_TIME = 3.0
CTRL_HZ           = 20

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
TRACK_DUR_S  = NUM_CYCLES * ROLL_PERIOD_S
TOTAL_PATH_M = PATH_SPEED_MPS * TRACK_DUR_S
OMEGA        = 2.0 * math.pi / ROLL_PERIOD_S

_HOVER_THR  = (1.4 * 9.81) / 24.0
_SAT_LIMIT  = math.degrees(math.asin(0.3 / _HOVER_THR))    # ≈ 31.6°

# ─────────────────────────────────────────────────────────────
# Shared setpoint state (Timer publishes, main thread updates)
# ─────────────────────────────────────────────────────────────
_sp      = {'x': 0.0, 'y': 0.0, 'z': TARGET_ALT_M, 'yaw': 0.0}
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
_imu_data      = Imu()


def _cb_state(msg): global _vehicle_state; _vehicle_state = msg
def _cb_pose(msg):  global _local_pose;    _local_pose    = msg
def _cb_imu(msg):   global _imu_data;      _imu_data      = msg


def get_altitude():
    return _local_pose.pose.position.z

def get_x():
    return _local_pose.pose.position.x

def get_roll_deg():
    o = _imu_data.orientation
    roll, _, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(roll)


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
rospy.init_node('pfa_roll_forward', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

sp_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

_sp_timer = rospy.Timer(
    rospy.Duration(1.0 / CTRL_HZ),
    lambda e: sp_pub.publish(_make_sp_msg()))

rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected. mode={_vehicle_state.mode}")
rospy.loginfo(f"  Roll: ±{ROLL_AMP_DEG:.0f}° × {NUM_CYCLES} cycles  "
              f"({'OK' if ROLL_AMP_DEG <= _SAT_LIMIT else f'WARNING > {_SAT_LIMIT:.1f}°'})")
rospy.loginfo(f"  Path: {PATH_SPEED_MPS} m/s × {TRACK_DUR_S:.0f} s = {TOTAL_PATH_M:.1f} m (ENU East)")
rospy.loginfo("  Syncing parameter cache from FCU...")
param_pull()

# ─────────────────────────────────────────────────────────────
# Step 0 – PFA parameters
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0°...")
rospy.loginfo("  PFA_DES_ROLL  = 0° ... " +
              ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH = 0° ... " +
              ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1 – Stream setpoints (OFFBOARD precondition)
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming setpoints (5 s, z={TARGET_ALT_M} m, yaw=0°)...")
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
# Step 3 – Stable hover
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Waiting for stable hover at {TARGET_ALT_M} m...")

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()

while not rospy.is_shutdown():
    alt     = get_altitude()
    alt_err = TARGET_ALT_M - alt
    now     = rospy.Time.now()

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
# Step 4 – Level hover hold
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Level hover hold for {HOVER_PHASE_DUR:.0f} s...")
t_end    = rospy.Time.now() + rospy.Duration(HOVER_PHASE_DUR)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        rospy.loginfo(f"  alt={get_altitude():.2f} m  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

path_start_x = get_x()
rospy.loginfo(f"  Path start x = {path_start_x:.2f} m (ENU East)")

# ─────────────────────────────────────────────────────────────
# Step 5 – Forward + roll oscillation
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Forward + roll ±{ROLL_AMP_DEG:.0f}°: "
              f"{NUM_CYCLES} cycles × {ROLL_PERIOD_S:.0f} s = {TRACK_DUR_S:.0f} s  "
              f"({TOTAL_PATH_M:.1f} m)")
rospy.loginfo(f"  XY saturation ≈ ±{_SAT_LIMIT:.1f}°  "
              f"({'OK' if ROLL_AMP_DEG <= _SAT_LIMIT else 'WARNING: may saturate'})")
rospy.loginfo(f"  {'Time':>6}  {'Xcmd':>6}  {'Xnow':>6}  {'Alt':>5}  "
              f"{'RollCmd':>8}  {'RollNow':>8}")
rospy.loginfo("  " + "─" * 54)

t_track    = rospy.Time.now()
last_param = rospy.Time.now()
last_log   = rospy.Time.now()
roll_cmd   = 0.0

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_track).to_sec()
    if elapsed >= TRACK_DUR_S:
        elapsed = TRACK_DUR_S

    x_cmd    = path_start_x + elapsed * PATH_SPEED_MPS
    roll_cmd = ROLL_AMP_DEG * math.sin(OMEGA * elapsed)

    update_sp(x=x_cmd, y=0.0, z=TARGET_ALT_M, yaw=0.0)

    # param update at ~5 Hz (blocking; Timer keeps setpoints flowing)
    if (now - last_param).to_sec() >= 0.2:
        param_set('PFA_DES_ROLL', roll_cmd, retries=1)
        last_param = rospy.Time.now()

    if (now - last_log).to_sec() >= 0.5:
        rospy.loginfo(f"  {elapsed:6.1f}s  {x_cmd:6.2f}m  {get_x():6.2f}m  "
                      f"{get_altitude():5.2f}m  {roll_cmd:8.1f}°  {get_roll_deg():8.1f}°")
        last_log = now

    if elapsed >= TRACK_DUR_S:
        break
    rate.sleep()

rospy.loginfo("\n  Roll forward complete.")

# ─────────────────────────────────────────────────────────────
# Step 6 – Reset PFA_DES_ROLL + hold end position
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 6] Resetting PFA_DES_ROLL = 0°...")
param_set('PFA_DES_ROLL', 0.0)

x_end = get_x()
update_sp(x=x_end, y=0.0)
rospy.loginfo(f"  Holding x ≈ {x_end:.2f} m for 5 s...")
t_end = rospy.Time.now() + rospy.Duration(5.0)
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 7 – Land
# ─────────────────────────────────────────────────────────────
_sp_timer.shutdown()
rospy.loginfo("\n[Step 7] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
