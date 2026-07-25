#!/usr/bin/env python3
"""
PFA Five-Pointed Star Path Tracking (MAVROS / ROS1 Noetic)
===========================================================
Task: 繞五角星軌跡飛行，roll=pitch=0°，yaw 朝向當前段落前進方向

五角星幾何（以懸停位置為中心，ENU）：
  外接圓半徑 R = STAR_RADIUS_M
  5 個頂點 (CCW，從北方頂點出發)：
    v[k] = (cx + R·cos(π/2 + k·2π/5),
             cy + R·sin(π/2 + k·2π/5))   k = 0..4

    v[0] = North (北)    v[1] = NW    v[2] = SW
    v[3] = SE            v[4] = NE

  Skip-one 順序描繪五角星（每次跨越2頂點）：
    0 → 2 → 4 → 1 → 3 → 0  (5段，每段長 = 2R·sin 72° ≈ 1.902R)

  各段偏航角（固定）：
    v[0]→v[2]: −108°   v[2]→v[4]: +36°   v[4]→v[1]: +180°
    v[1]→v[3]:  −36°   v[3]→v[0]: +108°

  各頂點偏航轉角：全部 = 144° (五角星幾何性質)

  頂點停留期間 (VERTEX_DWELL_S)：偏航命令切換為下一段方向，
  讓 PX4 完成 144° 的偏航旋轉後再出發。

執行方式：
  python3 pfa_star_mavros.py  (ROS 環境已 source)
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
TARGET_ALT_M      = 5.0    # 飛行高度 (m，ENU z+)
STAR_RADIUS_M     = 12.0   # 五角星外接圓半徑 (m)
SEG_SPEED_MPS     = 1.5    # 沿星邊飛行速度 (m/s)
VERTEX_DWELL_S    = 2.0    # 各頂點停留時間 (s)；PX4 需要約 1.6 s 完成 144° 偏航
NUM_LAPS          = 2      # 完整五角星飛行圈數
YAW_TRACK_PATH    = True   # True: yaw 追蹤當前段方向; False: 固定 yaw=0° (ENU East)

HOVER_PHASE_DUR   = 8.0
ALT_TOL           = 0.15
HOVER_STABLE_TIME = 3.0
CTRL_HZ           = 20

# ─────────────────────────────────────────────────────────────
# Derived: star geometry
# ─────────────────────────────────────────────────────────────
_STAR_VISIT = [0, 2, 4, 1, 3]                              # skip-one 順序
_SEGMENTS   = [(_STAR_VISIT[i], _STAR_VISIT[(i+1) % 5])   # 5 段
               for i in range(5)]

_SEG_LEN    = 2.0 * STAR_RADIUS_M * math.sin(math.radians(72.0))  # ≈ 22.83 m
_SEG_DUR_S  = _SEG_LEN / SEG_SPEED_MPS
_LAP_DUR_S  = 5 * (_SEG_DUR_S + VERTEX_DWELL_S)
_TOTAL_DUR_S = NUM_LAPS * _LAP_DUR_S


def _build_verts(cx, cy, R):
    """5 outer vertices of pentagram, CCW from top (North), ENU."""
    return [(cx + R * math.cos(math.pi / 2 + k * 2 * math.pi / 5),
             cy + R * math.sin(math.pi / 2 + k * 2 * math.pi / 5))
            for k in range(5)]


def _seg_yaw(verts, src_i, dst_i):
    """ENU yaw pointing from src vertex to dst vertex."""
    dx = verts[dst_i][0] - verts[src_i][0]
    dy = verts[dst_i][1] - verts[src_i][1]
    return math.atan2(dy, dx)


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
    """ENU PoseStamped，orientation quaternion 帶 yaw（防 NaN）。"""
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
rospy.init_node('pfa_star', anonymous=False)
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
rospy.loginfo(f"  Five-pointed star mission plan:")
rospy.loginfo(f"    Alt           : {TARGET_ALT_M:.0f} m (constant)")
rospy.loginfo(f"    Outer radius  : {STAR_RADIUS_M:.1f} m")
rospy.loginfo(f"    Segment length: {_SEG_LEN:.2f} m  (= 2R·sin 72°)")
rospy.loginfo(f"    Seg speed     : {SEG_SPEED_MPS:.1f} m/s  → {_SEG_DUR_S:.1f} s/seg")
rospy.loginfo(f"    Vertex dwell  : {VERTEX_DWELL_S:.1f} s  (yaw turn 144°)")
rospy.loginfo(f"    Laps          : {NUM_LAPS}  "
              f"(≈ {_LAP_DUR_S:.0f} s/lap × {NUM_LAPS} = {_TOTAL_DUR_S:.0f} s)")
rospy.loginfo(f"    Yaw mode      : "
              f"{'追蹤前進方向' if YAW_TRACK_PATH else '固定 0° (ENU East)'}")
rospy.loginfo(f"    Tracing order : v[0(N)] → v[2(SW)] → v[4(NE)] → "
              f"v[1(NW)] → v[3(SE)] → v[0(N)]")
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
# Step 3 – Hover hold; record star centre
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover hold for {HOVER_PHASE_DUR:.0f} s (recording star centre)...")
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
verts = _build_verts(cx, cy, STAR_RADIUS_M)

rospy.loginfo(f"  Star centre : ENU ({cx:.2f}, {cy:.2f}) m")
rospy.loginfo(f"  5 vertices  :")
label = ['N(top)', 'NW', 'SW', 'SE', 'NE']
for k, (vx, vy) in enumerate(verts):
    rospy.loginfo(f"    v[{k}] {label[k]:6s}: ({vx:.2f}, {vy:.2f}) m")

# ─────────────────────────────────────────────────────────────
# Step 4 – Approach v[0] (North tip) from centre
# ─────────────────────────────────────────────────────────────
v0x, v0y = verts[0]
APPROACH_DUR_S = 4.0
depart_yaw = _seg_yaw(verts, *_SEGMENTS[0]) if YAW_TRACK_PATH else 0.0

rospy.loginfo(f"\n[Step 4] Approaching v[0] (North tip) in {APPROACH_DUR_S:.0f} s...")
t_approach = rospy.Time.now()
while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_approach).to_sec()
    frac    = min(elapsed / APPROACH_DUR_S, 1.0)
    s       = frac * frac * (3.0 - 2.0 * frac)    # cubic ease-in-out
    sp_pub.publish(make_setpoint(cx + s * (v0x - cx),
                                 cy + s * (v0y - cy),
                                 TARGET_ALT_M, yaw_rad=depart_yaw))
    if frac >= 1.0:
        break
    rate.sleep()

rospy.loginfo(f"  At v[0] ({v0x:.1f}, {v0y:.1f}) m.")

# ─────────────────────────────────────────────────────────────
# Step 5 – Five-pointed star trajectory
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Star trajectory: {NUM_LAPS} lap(s) × 5 segs × "
              f"({_SEG_DUR_S:.1f}+{VERTEX_DWELL_S:.0f}) s ≈ {_TOTAL_DUR_S:.0f} s total")

for lap in range(NUM_LAPS):
    rospy.loginfo(f"\n  ══ Lap {lap + 1}/{NUM_LAPS} ══")

    for seg_idx, (src_i, dst_i) in enumerate(_SEGMENTS):
        src_x, src_y = verts[src_i]
        dst_x, dst_y = verts[dst_i]
        dx       = dst_x - src_x
        dy       = dst_y - src_y
        seg_yaw  = _seg_yaw(verts, src_i, dst_i) if YAW_TRACK_PATH else 0.0

        # Next segment's departure yaw (for dwell phase)
        next_src_i, next_dst_i = _SEGMENTS[(seg_idx + 1) % 5]
        next_yaw = _seg_yaw(verts, next_src_i, next_dst_i) if YAW_TRACK_PATH else 0.0

        rospy.loginfo(f"\n  ── Seg {seg_idx+1}/5  "
                      f"v[{src_i}]({label[src_i]}) → v[{dst_i}]({label[dst_i]})  "
                      f"yaw={math.degrees(seg_yaw):.0f}°  len={_SEG_LEN:.1f} m  "
                      f"dur={_SEG_DUR_S:.1f} s ──")
        rospy.loginfo(f"  {'Elapsed':>8}  {'Frac':>5}  {'Xcmd':>7}  {'Ycmd':>7}  "
                      f"{'Xnow':>7}  {'Ynow':>7}  {'Alt':>5}")
        rospy.loginfo("  " + "─" * 58)

        # ── Fly segment ──────────────────────────────────────
        t_seg    = rospy.Time.now()
        last_log = rospy.Time.now()

        while not rospy.is_shutdown():
            now     = rospy.Time.now()
            elapsed = (now - t_seg).to_sec()
            frac    = min(elapsed / _SEG_DUR_S, 1.0)
            x_cmd   = src_x + frac * dx
            y_cmd   = src_y + frac * dy

            sp_pub.publish(make_setpoint(x_cmd, y_cmd, TARGET_ALT_M, yaw_rad=seg_yaw))

            if (now - last_log).to_sec() > 1.0:
                xn, yn, _ = get_xyz()
                rospy.loginfo(f"  {elapsed:8.1f}s  {frac:5.2f}  "
                              f"{x_cmd:7.2f}m  {y_cmd:7.2f}m  "
                              f"{xn:7.2f}m  {yn:7.2f}m  {get_altitude():5.2f}m")
                last_log = now

            if frac >= 1.0:
                break
            rate.sleep()

        # ── Dwell at vertex (yaw rotates to next segment direction) ──
        rospy.loginfo(f"  v[{dst_i}] reached. Dwell {VERTEX_DWELL_S:.0f} s, "
                      f"yaw → {math.degrees(next_yaw):.0f}° (Δ=144°)")
        t_dwell = rospy.Time.now()
        while not rospy.is_shutdown() and \
              (rospy.Time.now() - t_dwell).to_sec() < VERTEX_DWELL_S:
            sp_pub.publish(make_setpoint(dst_x, dst_y, TARGET_ALT_M, yaw_rad=next_yaw))
            rate.sleep()

rospy.loginfo(f"\n  Star trajectory complete ({NUM_LAPS} lap(s)).")

# ─────────────────────────────────────────────────────────────
# Step 6 – Return to centre and hold
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Returning to star centre ({cx:.1f}, {cy:.1f}) m, hold 5 s...")
RETURN_DUR_S = 4.0
t_return = rospy.Time.now()
while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_return).to_sec()
    frac    = min(elapsed / RETURN_DUR_S, 1.0)
    s       = frac * frac * (3.0 - 2.0 * frac)
    sp_pub.publish(make_setpoint(v0x + s * (cx - v0x),
                                 v0y + s * (cy - v0y),
                                 TARGET_ALT_M, yaw_rad=0.0))
    if frac >= 1.0:
        break
    rate.sleep()

t_end    = rospy.Time.now() + rospy.Duration(5.0)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    sp_pub.publish(make_setpoint(cx, cy, TARGET_ALT_M, yaw_rad=0.0))
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 1.0:
        xn, yn, _ = get_xyz()
        r_deg, p_deg, _ = get_roll_pitch_yaw_deg()
        rospy.loginfo(f"  pos=({xn:.2f}, {yn:.2f}) m  alt={get_altitude():.2f} m  "
                      f"roll={r_deg:.1f}°  pitch={p_deg:.1f}°")
        last_log = now
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 7 – Land
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 7] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
