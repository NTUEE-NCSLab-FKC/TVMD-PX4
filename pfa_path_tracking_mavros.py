#!/usr/bin/env python3
"""
PFA Path Tracking (MAVROS / ROS1 Noetic)
==========================================
MAVROS 版本，對應 pfa_path_tracking.py (pymavlink 版)

Task: 直線前進，先 roll ±ROLL_AMPLITUDE° × N 週期，再 pitch ±PITCH_AMPLITUDE° × N 週期

軌跡 (ENU，MAVROS)：
  x = x_start + t × PATH_SPEED_MPS   (朝東前進，yaw=0°)
  y = 0                               (側向保持)
  z = TARGET_ALT_M                    (高度保持)
  yaw = 0°                            (朝東，ENU 約定)

控制架構：
  rospy.Timer (20 Hz) → /mavros/setpoint_position/local (PoseStamped)
                          ↓ MAVROS ENU→NED 轉換
  主執行緒:  /mavros/param/set PFA_DES_ROLL / PFA_DES_PITCH = A × sin(2π t / T)
                          ↓
  PX4:  pfa_pos_control → thrust (跟隨 x_cmd 位置)
        pfa_att_control → roll / pitch (跟隨振盪波形)

與 pymavlink 版的關鍵差異：
  - rospy.Timer 在獨立執行緒以 20 Hz 發佈 setpoint，主執行緒可安全阻塞在 param service
  - 座標系：ENU (x=東, y=北, z=上)；yaw=0° 朝東
  - Param 更新：/mavros/param/set service（比 pymavlink param_set_async 慢，~5 Hz）

執行方式：
  python3 pfa_path_tracking_mavros.py  (ROS 環境已 source)
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
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M         = 1.0    # 懸停高度 (m，ENU: z+ 方向)
PATH_SPEED_MPS       = 0.5    # 前進速度 (m/s，ENU x+)

ROLL_AMPLITUDE_DEG   = 45.0   # roll 振盪幅度 (deg)；飽和限制 ≈ ±31.6°
PITCH_AMPLITUDE_DEG  = 45.0   # pitch 振盪幅度 (deg)
OSCILLATION_PERIOD_S = 6.0    # 振盪週期 (s/cycle)
NUM_ROLL_CYCLES      = 2      # roll 完整振盪次數
NUM_PITCH_CYCLES     = 2      # pitch 完整振盪次數

HOVER_PHASE_DUR      = 8.0
ALT_TOL              = 0.15
HOVER_STABLE_TIME    = 3.0

CTRL_HZ              = 20     # 主迴路頻率 (Hz)；Timer 以此頻率發佈 setpoint

# ─────────────────────────────────────────────────────────────
# Derived constants
# ─────────────────────────────────────────────────────────────
ROLL_PHASE_DUR  = NUM_ROLL_CYCLES  * OSCILLATION_PERIOD_S
PITCH_PHASE_DUR = NUM_PITCH_CYCLES * OSCILLATION_PERIOD_S
TOTAL_PATH_M    = PATH_SPEED_MPS * (ROLL_PHASE_DUR + PITCH_PHASE_DUR)

_HOVER_THR  = (1.4 * 9.81) / 24.0
_SAT_LIMIT  = math.degrees(math.asin(0.3 / _HOVER_THR))    # ≈ 31.6°

# ─────────────────────────────────────────────────────────────
# Shared setpoint state (updated by main thread, published by Timer)
# ─────────────────────────────────────────────────────────────
_sp = {'x': 0.0, 'y': 0.0, 'z': TARGET_ALT_M, 'yaw': 0.0}  # ENU
_sp_lock = threading.Lock()


def update_sp(x=None, y=None, z=None, yaw=None):
    """Thread-safe update of the current position/yaw setpoint."""
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


def get_x():
    return _local_pose.pose.position.x


def get_roll_pitch_deg():
    """(roll_deg, pitch_deg) from IMU."""
    o = _imu_data.orientation
    roll, pitch, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(roll), math.degrees(pitch)


# ─────────────────────────────────────────────────────────────
# MAVROS service wrappers
# ─────────────────────────────────────────────────────────────

def param_set(name, value, retries=5):
    """Blocking param set (OK to call from main thread; Timer handles publishing)."""
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
# ROS init + Timer
# ─────────────────────────────────────────────────────────────
rospy.init_node('pfa_path_tracking', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

sp_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# Timer publishes at 20 Hz in a separate thread; main thread can safely block.
_sp_timer = rospy.Timer(
    rospy.Duration(1.0 / CTRL_HZ),
    lambda e: sp_pub.publish(_make_sp_msg()))

rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected. FCU status={_vehicle_state.system_status}")
rospy.loginfo(f"  Roll:  ±{ROLL_AMPLITUDE_DEG:.0f}°  "
              f"{'OK' if ROLL_AMPLITUDE_DEG  <= _SAT_LIMIT else f'WARNING > {_SAT_LIMIT:.1f}°'}")
rospy.loginfo(f"  Pitch: ±{PITCH_AMPLITUDE_DEG:.0f}°  "
              f"{'OK' if PITCH_AMPLITUDE_DEG <= _SAT_LIMIT else f'WARNING > {_SAT_LIMIT:.1f}°'}")
rospy.loginfo(f"  Path:  {PATH_SPEED_MPS} m/s × {ROLL_PHASE_DUR+PITCH_PHASE_DUR:.0f} s"
              f" = {TOTAL_PATH_M:.1f} m (ENU x+, East)")


# ─────────────────────────────────────────────────────────────
# Core oscillation loop
# ─────────────────────────────────────────────────────────────

def run_oscillation_phase(param_name, amplitude_deg, phase_dur, path_start_x):
    """
    直線前進同時對 param_name 施加正弦振盪。

    rospy.Timer 以 20 Hz 發佈 setpoint（主執行緒無需自行發布）；
    param_set 服務呼叫在主執行緒阻塞，約 5 Hz。
    返回本階段結束時的實際 x 座標。
    """
    label = 'roll' if 'ROLL' in param_name else 'pitch'
    t_start    = rospy.Time.now()
    last_param = rospy.Time.now()
    last_print = rospy.Time.now()

    rospy.loginfo(f"  {'Time':>6}  {'X_cmd':>6}  {'X_now':>6}  {'Alt':>5}  "
                  f"{label+'_cmd':>10}  {label+'_now':>10}")
    rospy.loginfo("  " + "─" * 56)

    while not rospy.is_shutdown():
        now     = rospy.Time.now()
        elapsed = (now - t_start).to_sec()
        if elapsed >= phase_dur:
            elapsed = phase_dur

        # ── update position setpoint (Timer will publish at 20 Hz) ──
        x_cmd = path_start_x + elapsed * PATH_SPEED_MPS
        update_sp(x=x_cmd, y=0.0, z=TARGET_ALT_M, yaw=0.0)

        # ── param update at ~5 Hz (blocking; Timer keeps setpoints flowing) ──
        if (now - last_param).to_sec() >= 0.2:
            angle_cmd = amplitude_deg * math.sin(
                2.0 * math.pi * elapsed / OSCILLATION_PERIOD_S)
            param_set(param_name, angle_cmd, retries=1)
            last_param = rospy.Time.now()

        # ── logging at 2 Hz ──────────────────────────────────────────
        if (now - last_print).to_sec() >= 0.5:
            angle_cmd_log = amplitude_deg * math.sin(
                2.0 * math.pi * elapsed / OSCILLATION_PERIOD_S)
            x_now         = get_x()
            alt           = get_altitude()
            r_deg, p_deg  = get_roll_pitch_deg()
            fb_deg = r_deg if 'ROLL' in param_name else p_deg
            rospy.loginfo(f"  {elapsed:6.1f}s  {x_cmd:6.2f}m  {x_now:6.2f}m  "
                          f"{alt:5.2f}m  {angle_cmd_log:9.1f}°  {fb_deg:9.1f}°")
            last_print = rospy.Time.now()

        if elapsed >= phase_dur:
            break
        rate.sleep()

    return get_x()


# ─────────────────────────────────────────────────────────────
# Step 0 – 起飛前將 PFA_DES_ROLL / PFA_DES_PITCH 歸零
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Initializing PFA attitude parameters...")
rospy.loginfo("  PFA_DES_ROLL  = 0° ... " +
              ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH = 0° ... " +
              ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1 – 串流初始 setpoint（PX4 進入 OFFBOARD 前的必要條件）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming initial setpoints (5 s, z={TARGET_ALT_M} m, yaw=0°)...")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 5.0:
    rate.sleep()    # Timer is already publishing

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
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 3 – 位置控制懸停至 TARGET_ALT_M
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover at {TARGET_ALT_M} m (PFA_DES_ROLL/PITCH = 0°)...")
rospy.loginfo(f"  Waiting for stable hover (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()

while not rospy.is_shutdown():
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
# Step 5 – Roll oscillation + straight-line forward
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Roll oscillation: ±{ROLL_AMPLITUDE_DEG:.0f}° × {NUM_ROLL_CYCLES} cycles  "
              f"[{ROLL_PHASE_DUR:.0f} s, Δx = {PATH_SPEED_MPS*ROLL_PHASE_DUR:.1f} m]")
rospy.loginfo(f"  XY saturation limit ≈ ±{_SAT_LIMIT:.1f}°  "
              f"({'OK' if ROLL_AMPLITUDE_DEG <= _SAT_LIMIT else 'WARNING: may saturate XY thrust'})")
rospy.loginfo(f"  PFA_DES_ROLL = {ROLL_AMPLITUDE_DEG:.0f} × sin(2π t / {OSCILLATION_PERIOD_S:.0f} s)")

current_x = run_oscillation_phase(
    'PFA_DES_ROLL', ROLL_AMPLITUDE_DEG, ROLL_PHASE_DUR, path_start_x)

rospy.loginfo(f"\n  Roll phase done. x = {current_x:.2f} m")
rospy.loginfo("  Resetting PFA_DES_ROLL = 0°...")
param_set('PFA_DES_ROLL', 0.0)
# brief level pause; Timer keeps publishing, update_sp holds position
update_sp(x=current_x)
rospy.sleep(2.0)

# ─────────────────────────────────────────────────────────────
# Step 6 – Pitch oscillation + continue straight-line forward
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Pitch oscillation: ±{PITCH_AMPLITUDE_DEG:.0f}° × {NUM_PITCH_CYCLES} cycles  "
              f"[{PITCH_PHASE_DUR:.0f} s, Δx = {PATH_SPEED_MPS*PITCH_PHASE_DUR:.1f} m]")
rospy.loginfo(f"  XY saturation limit ≈ ±{_SAT_LIMIT:.1f}°  "
              f"({'OK' if PITCH_AMPLITUDE_DEG <= _SAT_LIMIT else 'WARNING: may saturate XY thrust'})")
rospy.loginfo(f"  PFA_DES_PITCH = {PITCH_AMPLITUDE_DEG:.0f} × sin(2π t / {OSCILLATION_PERIOD_S:.0f} s)")

current_x = run_oscillation_phase(
    'PFA_DES_PITCH', PITCH_AMPLITUDE_DEG, PITCH_PHASE_DUR, current_x)

rospy.loginfo(f"\n  Pitch phase done. x = {current_x:.2f} m")
rospy.loginfo("  Resetting PFA_DES_PITCH = 0°...")
param_set('PFA_DES_PITCH', 0.0)

# ─────────────────────────────────────────────────────────────
# Step 7 – Hold end position for 10 s
# ─────────────────────────────────────────────────────────────
update_sp(x=current_x)
rospy.loginfo(f"\n[Step 7] Holding end position x ≈ {current_x:.1f} m for 10 s...")
t_end    = rospy.Time.now() + rospy.Duration(10.0)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        r_deg, p_deg = get_roll_pitch_deg()
        rospy.loginfo(f"  x={get_x():.2f} m  alt={get_altitude():.2f} m  "
                      f"roll={r_deg:.1f}°  pitch={p_deg:.1f}°  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 8 – Land
# ─────────────────────────────────────────────────────────────
_sp_timer.shutdown()
rospy.loginfo("\n[Step 8] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
