#!/usr/bin/env python3
"""
PFA Five-Pointed Star Path Tracking with Feedforward (MAVROS / ROS1 Noetic)
=============================================================================
Task: 繞五角星軌跡飛行，roll=pitch=0°，yaw 朝向當前段落前進方向，
      同時送出位置 + 速度 + 加速度前饋。pfa_star_mavros.py 的前饋版本。

■ 為什麼要前饋
  只送位置時 trajectory_setpoint.velocity/acceleration = NaN → pfa_pos_control
  清成 0，D 項以 0 為速度參考一直煞車 → 固定落後 (Kv/Kp)·速度。
  送出 v_ref、a_ref 後誤差動態為 ë + Kv·ė + Kp·e = 0，落後收斂到 0。
  （前饋無法消除重心偏移等固定干擾造成的穩態誤差，那需要積分項。）

■ 五角星幾何（以懸停位置為中心，ENU）
  外接圓半徑 R = STAR_RADIUS_M，5 個頂點 (CCW，從北方頂點出發)：
    v[k] = (cx + R·cos(π/2 + k·2π/5), cy + R·sin(π/2 + k·2π/5))   k = 0..4
  Skip-one 順序：0 → 2 → 4 → 1 → 3 → 0（5 段，每段長 = 2R·sin 72° ≈ 1.902R）
  每段使用最小 jerk 曲線（頂點處速度、加速度為 0），峰值速度 = SEG_PEAK_SPEED_MPS，
  每段時間 T = 1.875 × 段長 / 峰值速度；頂點停留 VERTEX_DWELL_S 完成 144° 偏航。

■ 控制鏈
  set_sp() → rospy.Timer (CTRL_HZ) → /mavros/setpoint_raw/local (PositionTarget)
  → SET_POSITION_TARGET_LOCAL_NED (p + v + a + yaw) → pfa_pos_control → pfa_att_control

執行方式：
  python3 pfa_star_ff_mavros.py  (ROS 環境已 source)
"""

import sys
import math
import threading

import rospy
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ParamValue, PositionTarget
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
    ParamPull,   ParamPullRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M       = 5.0    # 飛行高度 (m，ENU z+)
STAR_RADIUS_M      = 12.0   # 五角星外接圓半徑 (m)
SEG_PEAK_SPEED_MPS = 1.5    # 每段最小 jerk 曲線的峰值速度 (m/s)
VERTEX_DWELL_S     = 2.0    # 各頂點停留時間 (s)；PX4 需要約 1.6 s 完成 144° 偏航
NUM_LAPS           = 2      # 完整五角星飛行圈數
YAW_TRACK_PATH     = True   # True: yaw 追蹤當前段方向; False: 固定 yaw=0° (ENU East)

HOVER_PHASE_DUR    = 8.0
ALT_TOL            = 0.15
HOVER_STABLE_TIME  = 3.0
APPROACH_MIN_S     = 4.0    # 中心 ↔ v[0] 的最短移動時間 (s)

CTRL_HZ           = 20     # Timer 發布頻率（經 SiK 數傳時建議降到 10）
LOG_PERIOD_S      = 0.5    # 追蹤狀態印出間隔 (s)
PX4_SPEED_XY_MAX  = 1.0    # pfa_pos_control 的 _speed_xy_max (m/s)，水平速度參考上限
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × L / T

# PositionTarget.type_mask：位置、速度、加速度、yaw 都使用；只忽略 yaw_rate
# （pfa_pos_control / pfa_att_control 目前不讀 trajectory_setpoint.yawspeed）
TYPE_MASK_PVA_YAW = PositionTarget.IGNORE_YAW_RATE

ZERO3 = (0.0, 0.0, 0.0)

# Derived: star geometry
_STAR_VISIT = [0, 2, 4, 1, 3]                              # skip-one 順序
_SEGMENTS   = [(_STAR_VISIT[i], _STAR_VISIT[(i + 1) % 5])  # 5 段
               for i in range(5)]
_SEG_LEN    = 2.0 * STAR_RADIUS_M * math.sin(math.radians(72.0))
_SEG_DUR_S  = MIN_JERK_PEAK * _SEG_LEN / SEG_PEAK_SPEED_MPS
_LAP_DUR_S  = 5 * (_SEG_DUR_S + VERTEX_DWELL_S)
_TOTAL_DUR_S = NUM_LAPS * _LAP_DUR_S


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

def make_target(p, v=ZERO3, a=ZERO3, yaw_rad=0.0):
    """ENU PositionTarget：位置 + 速度 + 加速度 + yaw（全部為有限值）。

    三軸都必須是有限值：pfa_pos_control 的 _checkAllFinite() 只要任一分量
    為 NaN 就把整個向量清成 0。MAVROS 會把 ENU 轉成 NED 再送給 PX4。
    """
    sp = PositionTarget()
    sp.header.stamp     = rospy.Time.now()
    sp.header.frame_id  = 'map'
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    sp.type_mask        = TYPE_MASK_PVA_YAW
    sp.position.x, sp.position.y, sp.position.z = p
    sp.velocity.x, sp.velocity.y, sp.velocity.z = v
    sp.acceleration_or_force.x, sp.acceleration_or_force.y, sp.acceleration_or_force.z = a
    sp.yaw      = yaw_rad
    sp.yaw_rate = 0.0
    return sp


# ─────────────────────────────────────────────────────────────
# Shared setpoint（主執行緒寫入；Timer 背景讀取並發布）
# ─────────────────────────────────────────────────────────────
_sp_lock = threading.Lock()
_sp_msg  = [None]

def set_sp(msg):
    """執行緒安全地更新 setpoint（主執行緒呼叫）。"""
    with _sp_lock:
        _sp_msg[0] = msg

def _timer_cb(_event):
    """CTRL_HZ 背景回呼：更新 timestamp 後發布。"""
    with _sp_lock:
        msg = _sp_msg[0]
    if msg is not None:
        msg.header.stamp = rospy.Time.now()
        sp_pub.publish(msg)


# ─────────────────────────────────────────────────────────────
# Trajectory building blocks（每段回傳 fn(t) -> (p, v, a, yaw)，ENU）
# ─────────────────────────────────────────────────────────────

def min_jerk(t, T):
    """最小 jerk 曲線 s(t)∈[0,1]：回傳 (s, ṡ, s̈)，起訖速度與加速度皆為 0。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    u = t / T
    s   = u**3 * (10.0 - 15.0 * u + 6.0 * u * u)
    sd  = 30.0 * u * u * (1.0 - u) ** 2 / T
    sdd = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T)
    return s, sd, sdd


def line_duration(p0, p1, peak_speed, min_dur=0.0):
    """直線段時間：讓最小 jerk 曲線的峰值速度 = peak_speed（至少 min_dur）。"""
    dist = math.dist(p0, p1)
    return max(min_dur, MIN_JERK_PEAK * dist / peak_speed) if dist > 1e-6 else min_dur


def line_segment(p0, p1, T, yaw_rad):
    """p0 → p1 的直線段（最小 jerk），yaw 固定。"""
    d = tuple(b - a for a, b in zip(p0, p1))
    def fn(t):
        s, sd, sdd = min_jerk(t, T)
        return (tuple(a + s * k for a, k in zip(p0, d)),
                tuple(sd * k for k in d),
                tuple(sdd * k for k in d),
                yaw_rad)
    return fn


def run_trajectory(fn, duration, title):
    """以 fn(t) 送出 p/v/a/yaw 直到 duration，每 LOG_PERIOD_S 印出追蹤誤差。回傳終點狀態。"""
    rospy.loginfo(f"  ── {title}  ({duration:.1f} s) ──")
    t0       = rospy.Time.now()
    last_log = t0
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t   = min((now - t0).to_sec(), duration)
        p, v, a, yaw = fn(t)
        set_sp(make_target(p, v, a, yaw))

        if (now - last_log).to_sec() >= LOG_PERIOD_S:
            xn, yn, zn = get_xyz()
            rospy.loginfo(f"    {t:5.1f}s  cmd=({p[0]:6.2f},{p[1]:6.2f},{p[2]:5.2f})  "
                          f"pos=({xn:6.2f},{yn:6.2f},{zn:5.2f})  "
                          f"err={math.hypot(p[0] - xn, p[1] - yn):4.2f}m  "
                          f"|v_ff|={math.hypot(v[0], v[1]):4.2f}  |a_ff|={math.hypot(a[0], a[1]):4.2f}  "
                          f"yaw={math.degrees(yaw):6.1f}°")
            last_log = now
        if t >= duration:
            break
        rate.sleep()
    return fn(duration)


def hold(p, yaw_rad, duration, title):
    """在 p 以零速度/加速度前饋保持 duration 秒。"""
    rospy.loginfo(f"  ── {title}  ({duration:.1f} s) ──")
    set_sp(make_target(p, yaw_rad=yaw_rad))
    t_end    = rospy.Time.now() + rospy.Duration(duration)
    last_log = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < t_end:
        now = rospy.Time.now()
        if (now - last_log).to_sec() >= 2.0:
            xn, yn, zn = get_xyz()
            r_deg, p_deg, _ = get_roll_pitch_yaw_deg()
            rospy.loginfo(f"    pos=({xn:.2f},{yn:.2f}) alt={zn:.2f}m  "
                          f"err={math.hypot(p[0] - xn, p[1] - yn):.2f}m  "
                          f"roll={r_deg:+.1f}°  pitch={p_deg:+.1f}°")
            last_log = now
        rate.sleep()


def warn_speed(peak_xy_mps, what):
    if peak_xy_mps > PX4_SPEED_XY_MAX:
        rospy.logwarn(f"  WARNING: {what} peak horizontal speed {peak_xy_mps:.2f} m/s > "
                      f"pfa_pos_control _speed_xy_max {PX4_SPEED_XY_MAX:.1f} m/s "
                      f"→ velocity feedforward will be clipped. Slow down or shrink the path.")


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
            if svc(req).success:
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
            if svc(SetModeRequest(custom_mode=mode_str)).mode_sent:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"set_mode({mode_str}) failed: {e}")
    return False


def arm_vehicle(retries=5):
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        for _ in range(retries):
            if svc(CommandBoolRequest(value=True)).success:
                return True
            rospy.sleep(0.3)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"arming failed: {e}")
    return False


# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('pfa_star_ff', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)

sp_pub = rospy.Publisher(
    '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

set_sp(make_target((0.0, 0.0, TARGET_ALT_M)))   # 初始 setpoint（Timer 啟動前先設定）
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_cb)

rospy.loginfo("Waiting for MAVROS FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected. mode={_vehicle_state.mode}")
rospy.loginfo(f"  Five-pointed star (feedforward: p + v + a):")
rospy.loginfo(f"    Alt {TARGET_ALT_M:.1f} m, R {STAR_RADIUS_M:.1f} m, segment {_SEG_LEN:.2f} m, "
              f"peak {SEG_PEAK_SPEED_MPS:.1f} m/s → {_SEG_DUR_S:.1f} s/seg")
rospy.loginfo(f"    Dwell {VERTEX_DWELL_S:.1f} s, {NUM_LAPS} laps ≈ {_TOTAL_DUR_S:.0f} s")
warn_speed(SEG_PEAK_SPEED_MPS, "star")
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
# Timer 在背景發布，rospy.sleep() 讓主執行緒等待即可
rospy.sleep(5.0)

# arm_vehicle()/set_mode() 阻塞時，Timer 仍在背景以 CTRL_HZ 發布
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
# Step 2 – Stable hover at TARGET_ALT_M
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 2] Stable hover at {TARGET_ALT_M} m...")
rospy.loginfo(f"  Waiting (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")
stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()
while not rospy.is_shutdown():
    now, alt = rospy.Time.now(), get_altitude()
    alt_err  = TARGET_ALT_M - alt
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
# Step 3 – Hover hold, record path origin
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover hold for {HOVER_PHASE_DUR:.0f} s (recording path origin)...")
rospy.sleep(HOVER_PHASE_DUR)
cx, cy, _ = get_xyz()
rospy.loginfo(f"  Path origin: ENU ({cx:.2f}, {cy:.2f}) m")

verts = [(cx + STAR_RADIUS_M * math.cos(math.pi / 2 + k * 2 * math.pi / 5),
          cy + STAR_RADIUS_M * math.sin(math.pi / 2 + k * 2 * math.pi / 5)) for k in range(5)]
label = ['N(top)', 'NW', 'SW', 'SE', 'NE']
for k, (vx, vy) in enumerate(verts):
    rospy.loginfo(f"    v[{k}] {label[k]:6s}: ({vx:.2f}, {vy:.2f}) m")

def vert3(i):
    return (verts[i][0], verts[i][1], TARGET_ALT_M)

def seg_yaw(src_i, dst_i):
    if not YAW_TRACK_PATH:
        return 0.0
    return math.atan2(verts[dst_i][1] - verts[src_i][1], verts[dst_i][0] - verts[src_i][0])

centre     = (cx, cy, TARGET_ALT_M)
depart_yaw = seg_yaw(*_SEGMENTS[0])

# ─────────────────────────────────────────────────────────────
# Step 4 – Approach v[0] (North tip) from centre
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Approaching v[0] (North tip)...")
T_app = line_duration(centre, vert3(0), SEG_PEAK_SPEED_MPS, min_dur=APPROACH_MIN_S)
run_trajectory(line_segment(centre, vert3(0), T_app, depart_yaw), T_app, "centre → v[0]")
hold(vert3(0), depart_yaw, VERTEX_DWELL_S, "v[0] dwell")

# ─────────────────────────────────────────────────────────────
# Step 5 – Five-pointed star trajectory
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Star trajectory: {NUM_LAPS} lap(s) × 5 segs ≈ {_TOTAL_DUR_S:.0f} s total")
for lap in range(NUM_LAPS):
    rospy.loginfo(f"\n  ══ Lap {lap + 1}/{NUM_LAPS} ══")
    for seg_idx, (src_i, dst_i) in enumerate(_SEGMENTS):
        yaw      = seg_yaw(src_i, dst_i)
        next_yaw = seg_yaw(*_SEGMENTS[(seg_idx + 1) % 5])
        run_trajectory(line_segment(vert3(src_i), vert3(dst_i), _SEG_DUR_S, yaw), _SEG_DUR_S,
                       f"Seg {seg_idx + 1}/5  v[{src_i}]({label[src_i]}) → v[{dst_i}]({label[dst_i]})  "
                       f"yaw={math.degrees(yaw):.0f}°")
        hold(vert3(dst_i), next_yaw, VERTEX_DWELL_S,
             f"v[{dst_i}] dwell, yaw → {math.degrees(next_yaw):.0f}°")
rospy.loginfo(f"\n  Star trajectory complete ({NUM_LAPS} lap(s)).")

# ─────────────────────────────────────────────────────────────
# Step 6 – Return to centre and hold
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Returning to star centre ({cx:.1f}, {cy:.1f}) m...")
T_ret = line_duration(vert3(0), centre, SEG_PEAK_SPEED_MPS, min_dur=APPROACH_MIN_S)
run_trajectory(line_segment(vert3(0), centre, T_ret, 0.0), T_ret, "v[0] → centre")
hold(centre, 0.0, 5.0, "centre hold")

# ─────────────────────────────────────────────────────────────
# Land
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Land] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
# 確認已離開 OFFBOARD 才停止 Timer，否則會先觸發 offboard lost → Position mode
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'AUTO.LAND':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  AUTO.LAND not confirmed, retrying...")
        set_mode('AUTO.LAND')
        break
    rate.sleep()
_sp_timer.shutdown()
rospy.sleep(15.0)
rospy.loginfo("Done.")
