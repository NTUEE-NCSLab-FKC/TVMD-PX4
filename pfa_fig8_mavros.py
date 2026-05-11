#!/usr/bin/env python3
"""
PFA Figure-8 Path Tracking (MAVROS / ROS1 Noetic)
===================================================
Task: 繞 8 字軌跡飛行，roll=pitch=0°，yaw 跟隨前進切線方向

軌跡 (ENU，以懸停位置為 8 字中心)：

  Lissajous 1:2 參數曲線：
    x(t) = cx + A × sin(ω t)           A = FIG8_LONG_RADIUS
    y(t) = cy + B × sin(2ω t)          B = FIG8_LONG_RADIUS / 2
    z(t) = TARGET_ALT_M                 (定高)
    ω    = 2π / PERIOD_S

  一個完整 8 字週期 T = PERIOD_S：
    t=0        → (cx, cy)      前進方向：NE（往右半圓出發）
    t=T/4      → (cx+A, cy)   最右點，前進方向：S
    t=T/2      → (cx, cy)     中心交叉，前進方向：NW（進入左半圓）
    t=3T/4     → (cx-A, cy)   最左點，前進方向：S
    t=T        → (cx, cy)     中心，前進方向：SE（完成一圈）

  速度（切線 yaw）：
    vx = A·ω·cos(ω t)
    vy = 2B·ω·cos(2ω t)
    yaw = atan2(vy, vx)   ← 在中心交叉點速度連續，無跳變

最大速度（中心交叉時）：
    |v_max| = ω × sqrt(A² + 4B²)  （當 B=A/2 → A·ω·√2）

控制架構：
  Python → /mavros/setpoint_position/local (PoseStamped, ENU)
         → orientation quaternion 帶 yaw（防 NaN）
  PX4:  pfa_pos_control → thrust（追蹤 8 字位置）
        pfa_att_control → roll=0°, pitch=0°, yaw 跟切線

與螺旋腳本的差異：
  - z 固定（定高 8 字），無爬升
  - 無 approach 相位（t=0 時曲線起點即為懸停中心）
  - 速度在中心交叉點連續（Lissajous 曲線特性），可直接 atan2 追蹤 yaw

執行方式：
  python3 pfa_fig8_mavros.py  (ROS 環境已 source)
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
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M        = 5.0    # 飛行高度 (m，ENU z+)
FIG8_LONG_RADIUS    = 10.0   # 8字長軸半徑 A (m); 短軸 B = A/2 自動計算
PERIOD_S            = 30.0   # 一個完整 8 字的時間 (s)
NUM_LAPS            = 3      # 飛行完整 8 字圈數
YAW_TRACK_PATH      = True   # True: yaw 跟切線方向; False: 固定 yaw=0° (ENU East)

HOVER_PHASE_DUR     = 8.0    # 起飛後定高懸停時間 (s)
ALT_TOL             = 0.15   # 穩定懸停高度容差 (m)
HOVER_STABLE_TIME   = 3.0    # 穩定判定時間 (s)
CTRL_HZ             = 20     # 主迴路頻率

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
FIG8_SHORT_RADIUS = FIG8_LONG_RADIUS / 2.0       # B = A/2
OMEGA             = 2.0 * math.pi / PERIOD_S     # rad/s
TOTAL_DUR_S       = NUM_LAPS * PERIOD_S

# Peak speed at center crossing: |v| = ω·sqrt(A² + (2B)²) = A·ω·√2  (when B=A/2)
V_CENTER_MPS = OMEGA * math.sqrt(FIG8_LONG_RADIUS**2 + (2*FIG8_SHORT_RADIUS)**2)
# Peak centripetal acceleration at lobe tips (x=±A):
#   speed = 2B·ω = A·ω, curvature ≈ 2B/A² = 1/A  → a = v²·κ (approximation)
V_LOBE_MPS = FIG8_SHORT_RADIUS * 2.0 * OMEGA     # speed at x=±A (vx=0)

# ─────────────────────────────────────────────────────────────
# Global state
# ─────────────────────────────────────────────────────────────
_vehicle_state = State()
_local_pose    = PoseStamped()
_imu_data      = Imu()


def _cb_state(msg): global _vehicle_state; _vehicle_state = msg
def _cb_pose(msg):  global _local_pose;    _local_pose    = msg
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

def make_setpoint(x, y, z, yaw_rad=0.0):
    """ENU PoseStamped; orientation quaternion 包含 yaw（防 NaN）。"""
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


def fig8_yaw(elapsed):
    """ENU yaw aligned to Lissajous tangent. Continuous at center crossings."""
    vx = FIG8_LONG_RADIUS  * OMEGA       * math.cos(OMEGA * elapsed)
    vy = FIG8_SHORT_RADIUS * 2.0 * OMEGA * math.cos(2.0 * OMEGA * elapsed)
    if abs(vx) < 1e-9 and abs(vy) < 1e-9:
        return 0.0
    return math.atan2(vy, vx)


# ─────────────────────────────────────────────────────────────
# MAVROS service wrappers
# ─────────────────────────────────────────────────────────────

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
rospy.init_node('pfa_fig8', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

sp_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected. mode={_vehicle_state.mode}")
rospy.loginfo(f"  Figure-8 mission plan:")
rospy.loginfo(f"    Alt           : {TARGET_ALT_M:.0f} m (constant)")
rospy.loginfo(f"    Long radius A : {FIG8_LONG_RADIUS:.1f} m  "
              f"(ENU x ± {FIG8_LONG_RADIUS:.0f} m)")
rospy.loginfo(f"    Short radius B: {FIG8_SHORT_RADIUS:.1f} m  "
              f"(ENU y ± {FIG8_SHORT_RADIUS:.0f} m)")
rospy.loginfo(f"    Period        : {PERIOD_S:.0f} s/lap × {NUM_LAPS} laps = "
              f"{TOTAL_DUR_S:.0f} s")
rospy.loginfo(f"    Speed (center): {V_CENTER_MPS:.2f} m/s  "
              f"(lobe tips: {V_LOBE_MPS:.2f} m/s)")
rospy.loginfo(f"    Yaw mode      : "
              f"{'追蹤切線方向 (atan2)' if YAW_TRACK_PATH else '固定 0° (ENU East)'}")

# ─────────────────────────────────────────────────────────────
# Step 0 – PFA attitude parameters
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0°...")
rospy.loginfo("  PFA_DES_ROLL  = 0° ... " +
              ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH = 0° ... " +
              ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming setpoints (5 s, z={TARGET_ALT_M} m)...")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 5.0:
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_ALT_M))
    rate.sleep()

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
# Step 2 – Stable hover at TARGET_ALT_M
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 2] Stable hover at {TARGET_ALT_M} m...")
rospy.loginfo(f"  Waiting (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()

while not rospy.is_shutdown():
    sp_pub.publish(make_setpoint(0.0, 0.0, TARGET_ALT_M))
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
# Step 3 – Level hover hold, record figure-8 center
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover hold for {HOVER_PHASE_DUR:.0f} s (recording 8-center)...")
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

cx, cy, _ = get_xyz()
rospy.loginfo(f"  Figure-8 center: ENU ({cx:.2f}, {cy:.2f}) m")
rospy.loginfo(f"  Right lobe: x ≈ [{cx:.1f}, {cx+FIG8_LONG_RADIUS:.1f}] m  "
              f"Left lobe: x ≈ [{cx-FIG8_LONG_RADIUS:.1f}, {cx:.1f}] m")

# ─────────────────────────────────────────────────────────────
# Step 4 – Figure-8 trajectory
# ─────────────────────────────────────────────────────────────
initial_yaw = fig8_yaw(0.0)   # NE direction at t=0

rospy.loginfo(f"\n[Step 4] Figure-8: {NUM_LAPS} laps × {PERIOD_S:.0f} s = {TOTAL_DUR_S:.0f} s")
rospy.loginfo(f"  Initial yaw command: {math.degrees(initial_yaw):.1f}°")
rospy.loginfo(f"  {'Time':>6}  {'Lap':>4}  {'Xcmd':>7}  {'Ycmd':>7}  "
              f"{'Xnow':>7}  {'Ynow':>7}  {'Alt':>5}  {'YawCmd':>8}")
rospy.loginfo("  " + "─" * 68)

t_track  = rospy.Time.now()
last_log = rospy.Time.now()
yaw_cmd  = initial_yaw

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_track).to_sec()
    if elapsed >= TOTAL_DUR_S:
        elapsed = TOTAL_DUR_S

    x_cmd = cx + FIG8_LONG_RADIUS  * math.sin(OMEGA * elapsed)
    y_cmd = cy + FIG8_SHORT_RADIUS * math.sin(2.0 * OMEGA * elapsed)

    if YAW_TRACK_PATH:
        yaw_cmd = fig8_yaw(elapsed)

    sp_pub.publish(make_setpoint(x_cmd, y_cmd, TARGET_ALT_M, yaw_rad=yaw_cmd))

    if (now - last_log).to_sec() >= 0.5:
        xn, yn, _ = get_xyz()
        lap_num   = elapsed / PERIOD_S
        rospy.loginfo(f"  {elapsed:6.1f}s  {lap_num:4.2f}  {x_cmd:7.2f}m  {y_cmd:7.2f}m  "
                      f"{xn:7.2f}m  {yn:7.2f}m  {get_altitude():5.2f}m  "
                      f"{math.degrees(yaw_cmd):7.1f}°")
        last_log = now

    if elapsed >= TOTAL_DUR_S:
        break
    rate.sleep()

rospy.loginfo("\n  Figure-8 complete.")

# ─────────────────────────────────────────────────────────────
# Step 5 – Return to center and hold
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Returning to center ({cx:.1f}, {cy:.1f}) m and holding 5 s...")
t_end    = rospy.Time.now() + rospy.Duration(5.0)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    sp_pub.publish(make_setpoint(cx, cy, TARGET_ALT_M, yaw_rad=0.0))
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 1.0:
        xn, yn, _ = get_xyz()
        r_deg, p_deg, _ = get_roll_pitch_yaw_deg()
        rospy.loginfo(f"  pos=({xn:.2f}, {yn:.2f})  alt={get_altitude():.2f} m  "
                      f"roll={r_deg:.1f}°  pitch={p_deg:.1f}°")
        last_log = now
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 6 – Land
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 6] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
