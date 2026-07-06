#!/usr/bin/env python3
"""
Velocity Profile Forward Flight (MAVROS / ROS 1)
=================================================
起飛懸停後，依序執行使用者定義的速度剖面向前飛行，
即時顯示各階段期望速度 vs 實際速度。

■ 控制鏈
  PositionTarget (velocity mode) → pfa_pos_control → pfa_att_control
  PFA_DES_PITCH=0  → 機體保持水平

■ 速度上限
  PFA_VEL_LIMIT 來自韌體原始碼：
    src/modules/pfa_pos_control/pfa_pos_control.hpp  line 116
      float _speed_xy_max{1.0f};

執行方式：
  python3 vel_profile_test_mavros.py
"""

import sys
import math
import threading
import rospy
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, ParamValue
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration ← 使用者修改這裡
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M  = 1.0     # 飛行高度 (m)
STOP_RAMP_S   = 2.0     # 結束後減速時間 (s)
CTRL_HZ       = 20      # Timer 發布頻率

# 速度剖面：[(速度 m/s, 持續秒數), ...]
# 每個 tuple = 一個飛行階段，依序執行
VEL_PROFILE = [
    (0.5, 10),   # Phase 1：0.5 m/s，10 秒
    (1.0, 10),   # Phase 2：1.0 m/s，10 秒
]

# pfa_pos_control 速度上限（韌體硬碼，不可透過 MAVLink 調整）
# src/modules/pfa_pos_control/pfa_pos_control.hpp  line 116
PFA_VEL_LIMIT = 1.0   # m/s

# ─────────────────────────────────────────────────────────────
# PositionTarget type_mask
# MASK_POS: position + yaw（起飛/懸停用）
# MASK_VEL: velocity + yaw（速度剖面用）
# ─────────────────────────────────────────────────────────────
MASK_POS = 8 + 16 + 32 + 64 + 128 + 256 + 2048    # = 2552
MASK_VEL = 1 + 2 + 4 + 64 + 128 + 256 + 2048      # = 2503

# ─────────────────────────────────────────────────────────────
# Subscribers
# ─────────────────────────────────────────────────────────────
_state       = State()
_pose        = PoseStamped()
_imu         = Imu()
_vel_stamped = TwistStamped()

def _cb_state(msg): global _state;        _state        = msg
def _cb_pose(msg):  global _pose;         _pose         = msg
def _cb_imu(msg):   global _imu;          _imu          = msg
def _cb_vel(msg):   global _vel_stamped;  _vel_stamped  = msg

def get_alt():  return _pose.pose.position.z
def get_xyz():
    p = _pose.pose.position; return p.x, p.y, p.z
def get_yaw_rad():
    o = _imu.orientation
    _, _, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return y
def get_rpy_deg():
    o = _imu.orientation
    r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(r), math.degrees(p), math.degrees(y)
def get_body_vel():
    """ENU 速度 → 機體前向 / 側向速度 (m/s)。"""
    vx = _vel_stamped.twist.linear.x
    vy = _vel_stamped.twist.linear.y
    yaw = get_yaw_rad()
    return (vx * math.cos(yaw) + vy * math.sin(yaw),
           -vx * math.sin(yaw) + vy * math.cos(yaw))

# ─────────────────────────────────────────────────────────────
# Shared PositionTarget setpoint（Timer 背景發布）
# ─────────────────────────────────────────────────────────────
_sp_lock = threading.Lock()
_sp_msg  = [None]

def set_sp(msg):
    with _sp_lock:
        _sp_msg[0] = msg

def _timer_cb(_event):
    with _sp_lock:
        msg = _sp_msg[0]
    if msg is not None:
        msg.header.stamp = rospy.Time.now()
        sp_pub.publish(msg)

def make_pos_sp(x, y, alt=None, yaw=0.0):
    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1
    msg.type_mask        = MASK_POS
    msg.position.x = x;  msg.position.y = y
    msg.position.z = alt if alt is not None else TARGET_ALT_M
    msg.yaw = yaw
    return msg

def make_vel_sp(vx_body=0.0, vy_body=0.0, vz=0.0):
    """機體 FLU 速度 → ENU world frame setpoint。"""
    yaw    = get_yaw_rad()
    vx_enu = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
    vy_enu = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1
    msg.type_mask        = MASK_VEL
    msg.velocity.x = vx_enu;  msg.velocity.y = vy_enu;  msg.velocity.z = vz
    msg.yaw = yaw
    return msg

# ─────────────────────────────────────────────────────────────
# Service helpers
# ─────────────────────────────────────────────────────────────
def param_set(name, value, retries=5):
    try:
        rospy.wait_for_service('/mavros/param/set', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        for _ in range(retries):
            if svc(req).success: return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"param_set({name}): {e}")
    return False

def set_mode(mode_str, retries=5):
    try:
        rospy.wait_for_service('/mavros/set_mode', timeout=5)
        svc = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        for _ in range(retries):
            if svc(SetModeRequest(custom_mode=mode_str)).mode_sent: return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"set_mode({mode_str}): {e}")
    return False

def arm_vehicle(retries=5):
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        for _ in range(retries):
            if svc(CommandBoolRequest(value=True)).success: return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"arming: {e}")
    return False

# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('vel_profile_test', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',                         State,        _cb_state)
rospy.Subscriber('/mavros/local_position/pose',           PoseStamped,  _cb_pose)
rospy.Subscriber('/mavros/imu/data',                      Imu,          _cb_imu)
rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, _cb_vel)

sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

set_sp(make_pos_sp(0.0, 0.0))
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_cb)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線 + 參數檢查
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _state.connected:
    rate.sleep()

total_dur = sum(d for _, d in VEL_PROFILE)
total_est = sum(v * d for v, d in VEL_PROFILE)

rospy.loginfo(f"  Connected.  mode={_state.mode}  armed={_state.armed}")
rospy.loginfo(f"  ┌─ Velocity Profile ─────────────────────────────┐")
for i, (v, d) in enumerate(VEL_PROFILE):
    flag = f"  ⚠ > {PFA_VEL_LIMIT:.1f} m/s LIMIT" if v > PFA_VEL_LIMIT else ""
    rospy.loginfo(f"  │  Phase {i+1}: {v:5.2f} m/s × {d:5.1f} s "
                  f"= {v*d:5.1f} m{flag}")
rospy.loginfo(f"  │  Total  : {'':5s}    {total_dur:5.1f} s = {total_est:5.1f} m (est.)")
rospy.loginfo(f"  │  Alt    : {TARGET_ALT_M:.1f} m")
rospy.loginfo(f"  └────────────────────────────────────────────────┘")

# 超速警告
for i, (v, _) in enumerate(VEL_PROFILE):
    if v > PFA_VEL_LIMIT:
        rospy.logwarn(f"  Phase {i+1}: {v:.2f} m/s > pfa_pos_control 上限 "
                      f"{PFA_VEL_LIMIT:.1f} m/s → 實際速度會被截斷至 {PFA_VEL_LIMIT:.1f} m/s")

# ─────────────────────────────────────────────────────────────
# Step 0: PFA params
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
rospy.loginfo("  PFA_DES_ROLL  ... " + ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH ... " + ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1-3: Pre-stream → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 1] Pre-streaming setpoints (5 s)...")
rospy.sleep(5.0)

rospy.loginfo("\n[Step 2] Requesting OFFBOARD mode...")
set_mode('OFFBOARD')
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_state.mode}")

rospy.loginfo("\n[Step 3] Arming...")
arm_vehicle()
t0 = rospy.Time.now()
while not rospy.is_shutdown() and not _state.armed:
    if (rospy.Time.now() - t0).to_sec() > 10.0:
        rospy.logerr("  ✗ Failed to arm. Exiting.")
        _sp_timer.shutdown(); sys.exit(1)
    rate.sleep()
rospy.loginfo("  ✓ Armed!")

# ─────────────────────────────────────────────────────────────
# Step 4: 起飛至 TARGET_ALT_M
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Takeoff to {TARGET_ALT_M:.1f} m...")
set_sp(make_pos_sp(0.0, 0.0, TARGET_ALT_M))

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()
ALT_TOL      = 0.10

while not rospy.is_shutdown():
    now, alt = rospy.Time.now(), get_alt()
    err = TARGET_ALT_M - alt
    stable_since = (stable_since or now) if abs(err) < ALT_TOL else None
    if (now - last_log).to_sec() >= 1.0:
        s = (now - stable_since).to_sec() if stable_since else 0.0
        rospy.loginfo(f"  alt={alt:.2f}m  err={err:+.2f}m  stable={s:.1f}/3.0s")
        last_log = now
    if stable_since and (now - stable_since).to_sec() >= 3.0:
        rospy.loginfo(f"  ✓ Stable at {alt:.2f} m."); break
    if (now - t_phase).to_sec() > 20.0:
        rospy.logwarn(f"  Hover timeout, alt={alt:.2f}m, continuing."); break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 5: 速度剖面飛行
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Velocity profile flight ({len(VEL_PROFILE)} phase(s))...")

phase_results = []   # [(vel_cmd, dur, avg_actual_vel, dist_actual)]
global_t0 = rospy.Time.now()

for phase_idx, (vel_cmd, dur) in enumerate(VEL_PROFILE):
    clamped = min(vel_cmd, PFA_VEL_LIMIT)
    rospy.loginfo(f"\n  ══ Phase {phase_idx+1}/{len(VEL_PROFILE)}  "
                  f"vel={vel_cmd:.2f} m/s  dur={dur:.1f} s "
                  f"{'(clamped → ' + str(clamped) + ' m/s)' if vel_cmd > PFA_VEL_LIMIT else ''} ══")
    rospy.loginfo(f"  {'Time':>6}  {'Cmd':>6}  {'Actual':>7}  {'Error':>7}  "
                  f"{'Alt':>5}  {'Roll':>6}  {'Pitch':>6}")
    rospy.loginfo("  " + "─" * 54)

    x_start, _, _ = get_xyz()
    vel_samples    = []
    t0 = last_log  = rospy.Time.now()

    while not rospy.is_shutdown():
        now     = rospy.Time.now()
        elapsed = (now - t0).to_sec()

        set_sp(make_vel_sp(vx_body=vel_cmd))   # 每幀更新以追蹤最新 yaw

        vx_act, _ = get_body_vel()
        vel_samples.append(vx_act)

        if (now - last_log).to_sec() >= 0.5:
            r, p, _ = get_rpy_deg()
            err     = vel_cmd - vx_act
            rospy.loginfo(f"  {elapsed:6.1f}s  {vel_cmd:6.2f}  "
                          f"{vx_act:7.3f}  {err:+7.3f}  "
                          f"{get_alt():5.2f}m  {r:+6.1f}°  {p:+6.1f}°")
            last_log = now

        if elapsed >= dur:
            break
        rate.sleep()

    x_end, _, _ = get_xyz()
    avg_vel = sum(vel_samples) / len(vel_samples) if vel_samples else 0.0
    dist    = x_end - x_start
    phase_results.append((vel_cmd, dur, avg_vel, dist))
    rospy.loginfo(f"  ✓ Phase {phase_idx+1} done.  "
                  f"avg_actual={avg_vel:.3f} m/s  dist≈{dist:.2f} m")

# ─────────────────────────────────────────────────────────────
# Step 6: 減速停止
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Decelerating to 0 over {STOP_RAMP_S:.1f} s...")
last_vel = VEL_PROFILE[-1][0] if VEL_PROFILE else 0.0
t_stop   = rospy.Time.now()
while not rospy.is_shutdown():
    elapsed = (rospy.Time.now() - t_stop).to_sec()
    if elapsed >= STOP_RAMP_S: break
    set_sp(make_vel_sp(vx_body=last_vel * (1.0 - elapsed / STOP_RAMP_S)))
    rate.sleep()

cx, cy, cz = get_xyz()
set_sp(make_pos_sp(cx, cy, alt=cz))
rospy.loginfo(f"  Stopped at ({cx:.2f}, {cy:.2f}, {cz:.2f}) m")

# ─────────────────────────────────────────────────────────────
# 飛行摘要
# ─────────────────────────────────────────────────────────────
total_elapsed = (rospy.Time.now() - global_t0).to_sec()
rospy.loginfo(f"\n  ─── Flight Summary ────────────────────────────────────")
rospy.loginfo(f"  {'Phase':>5}  {'Cmd':>6}  {'Dur':>5}  "
              f"{'AvgActual':>9}  {'EstDist':>7}  {'ActDist':>7}")
rospy.loginfo(f"  " + "─" * 52)
for i, (v, d, avg, dist) in enumerate(phase_results):
    rospy.loginfo(f"  {i+1:>5}  {v:6.2f}  {d:5.1f}s  "
                  f"{avg:9.3f}  {v*d:7.2f}m  {dist:7.2f}m")
rospy.loginfo(f"  ─────────────────────────────────────────────────────")
rospy.loginfo(f"  Total  : cmd={sum(v*d for v,d,*_ in phase_results):.2f} m  "
              f"actual≈{sum(dist for _,_,_,dist in phase_results):.2f} m")
rospy.loginfo(f"  ──────────────────────────────────────────────────────")

# ─────────────────────────────────────────────────────────────
# Step 7: 懸停 3 s 後降落
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 7] Hover 3 s then AUTO.LAND...")
rospy.sleep(3.0)

_sp_timer.shutdown()
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
