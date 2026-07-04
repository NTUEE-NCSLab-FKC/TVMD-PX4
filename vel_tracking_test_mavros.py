#!/usr/bin/env python3
"""
Forward Velocity Tracking Test (MAVROS / ROS 1)
================================================
起飛懸停後向前飛行，即時顯示期望速度 vs 實際速度，
用於驗證 pfa_pos_control 速度追蹤表現。

■ pfa_pos_control 速度硬碼上限：1.0 m/s（不可透過參數調整）
  超過 1.0 m/s 的命令會被截斷，腳本會自動警告並 clamp。

■ 使用方式
  python3 vel_tracking_test_mavros.py                       # 預設 0.5 m/s, 10 s
  python3 vel_tracking_test_mavros.py --vel 0.8 --dur 15
  python3 vel_tracking_test_mavros.py --vel 1.0 --dur 20 --alt 1.2

■ 參數
  --vel  目標前進速度 m/s（預設 0.5，pfa_pos_control 上限 1.0）
  --dur  前進飛行時間 s（預設 10）
  --alt  飛行高度 m（預設 1.0）
"""

import sys
import math
import threading
import argparse
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
# CLI 參數（使用者設置）
# ─────────────────────────────────────────────────────────────
_parser = argparse.ArgumentParser(description='Forward velocity tracking test')
_parser.add_argument('--vel', type=float, default=0.5,
                     metavar='V', help='目標前進速度 m/s (預設 0.5, 上限 1.0)')
_parser.add_argument('--dur', type=float, default=10.0,
                     metavar='T', help='前進飛行時間 s (預設 10)')
_parser.add_argument('--alt', type=float, default=1.0,
                     metavar='Z', help='飛行高度 m (預設 1.0)')
_args, _ = _parser.parse_known_args()   # ignore ROS remapping args

PFA_VEL_LIMIT   = 1.0                              # pfa_pos_control 硬碼上限
TARGET_ALT_M    = _args.alt
FLY_DURATION_S  = _args.dur
DESIRED_VEL_MPS = _args.vel

STOP_RAMP_S     = 2.0    # 減速爬坡時間 (s)
HOVER_STABLE_S  = 3.0    # 起飛穩定判斷連續秒數
ALT_TOL         = 0.10   # 高度容忍 (m)
CTRL_HZ         = 20     # Timer 發布頻率

# PositionTarget type_mask（數值計算見 pfa_vel_test_mavros.py 注解）
MASK_POS = 8 + 16 + 32 + 64 + 128 + 256 + 2048    # position + yaw
MASK_VEL = 1 + 2 + 4 + 64 + 128 + 256 + 2048      # velocity + yaw

# ─────────────────────────────────────────────────────────────
# Subscribers
# ─────────────────────────────────────────────────────────────
_state = State()
_pose  = PoseStamped()
_imu   = Imu()
_vel_stamped = TwistStamped()

def _cb_state(msg): global _state;        _state        = msg
def _cb_pose(msg):  global _pose;         _pose         = msg
def _cb_imu(msg):   global _imu;          _imu          = msg
def _cb_vel(msg):   global _vel_stamped;  _vel_stamped  = msg

def get_alt():
    return _pose.pose.position.z

def get_xyz():
    p = _pose.pose.position
    return p.x, p.y, p.z

def get_yaw_rad():
    o = _imu.orientation
    _, _, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return y

def get_rpy_deg():
    o = _imu.orientation
    r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(r), math.degrees(p), math.degrees(y)

def get_body_vel():
    """ENU 世界座標速度 → 機體前向 / 側向速度 (m/s)。"""
    vx_enu = _vel_stamped.twist.linear.x
    vy_enu = _vel_stamped.twist.linear.y
    yaw    = get_yaw_rad()
    vx_body =  vx_enu * math.cos(yaw) + vy_enu * math.sin(yaw)
    vy_body = -vx_enu * math.sin(yaw) + vy_enu * math.cos(yaw)
    return vx_body, vy_body

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
    """位置模式 setpoint（懸停/起飛用）。"""
    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1
    msg.type_mask        = MASK_POS
    msg.position.x = x
    msg.position.y = y
    msg.position.z = alt if alt is not None else TARGET_ALT_M
    msg.yaw        = yaw
    return msg

def make_vel_sp(vx_body=0.0, vy_body=0.0, vz=0.0):
    """
    速度模式 setpoint：機體 FLU 速度轉換至 ENU world frame。
    vz=0 → 嘗試維持垂直速度為零（高度可能有小幅漂移）。
    """
    yaw    = get_yaw_rad()
    vx_enu = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
    vy_enu = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1
    msg.type_mask        = MASK_VEL
    msg.velocity.x = vx_enu
    msg.velocity.y = vy_enu
    msg.velocity.z = vz
    msg.yaw        = yaw
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
rospy.init_node('vel_tracking_test', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',                      State,        _cb_state)
rospy.Subscriber('/mavros/local_position/pose',        PoseStamped,  _cb_pose)
rospy.Subscriber('/mavros/imu/data',                   Imu,          _cb_imu)
rospy.Subscriber('/mavros/local_position/velocity_local',
                 TwistStamped, _cb_vel)

sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

set_sp(make_pos_sp(0.0, 0.0))
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_cb)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線 + 顯示任務參數
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _state.connected:
    rate.sleep()

if DESIRED_VEL_MPS > PFA_VEL_LIMIT:
    rospy.logwarn(f"  ⚠ --vel {DESIRED_VEL_MPS:.2f} > pfa_pos_control 上限 "
                  f"{PFA_VEL_LIMIT:.1f} m/s，已 clamp 至 {PFA_VEL_LIMIT:.1f}")
    DESIRED_VEL_MPS = PFA_VEL_LIMIT

rospy.loginfo(f"  Connected.  mode={_state.mode}  armed={_state.armed}")
rospy.loginfo(f"  ┌─ Mission Parameters ─────────────────┐")
rospy.loginfo(f"  │  Target velocity : {DESIRED_VEL_MPS:>6.2f} m/s          │")
rospy.loginfo(f"  │  Flight duration : {FLY_DURATION_S:>6.1f} s            │")
rospy.loginfo(f"  │  Target altitude : {TARGET_ALT_M:>6.2f} m            │")
rospy.loginfo(f"  │  Est. distance   : {DESIRED_VEL_MPS * FLY_DURATION_S:>6.1f} m            │")
rospy.loginfo(f"  └──────────────────────────────────────┘")

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
while not rospy.is_shutdown():
    now, alt = rospy.Time.now(), get_alt()
    alt_err  = TARGET_ALT_M - alt
    stable_since = (stable_since or now) if abs(alt_err) < ALT_TOL else None

    if (now - last_log).to_sec() >= 1.0:
        s = (now - stable_since).to_sec() if stable_since else 0.0
        rospy.loginfo(f"  alt={alt:.2f}m  err={alt_err:+.2f}m  stable={s:.1f}/{HOVER_STABLE_S:.0f}s")
        last_log = now
    if stable_since and (now - stable_since).to_sec() >= HOVER_STABLE_S:
        rospy.loginfo(f"  ✓ Stable at {alt:.2f} m.")
        break
    if (now - t_phase).to_sec() > 20.0:
        rospy.logwarn(f"  Hover timeout alt={alt:.2f}m, continuing.")
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 5: 前進飛行（速度命令）
# 每幀更新以追蹤最新 yaw；即時記錄期望 vs 實際速度
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Forward flight: {DESIRED_VEL_MPS:.2f} m/s × {FLY_DURATION_S:.1f} s")
rospy.loginfo(f"  pfa_pos_control 上限: {PFA_VEL_LIMIT:.1f} m/s")
rospy.loginfo(f"  {'Time':>6}  {'Cmd':>6}  {'Actual':>7}  {'Error':>7}  "
              f"{'Alt':>5}  {'Roll':>6}  {'Pitch':>6}")
rospy.loginfo("  " + "─" * 56)

t0 = last_log = rospy.Time.now()
while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t0).to_sec()

    set_sp(make_vel_sp(vx_body=DESIRED_VEL_MPS))   # 每幀更新 yaw

    if (now - last_log).to_sec() >= 0.5:
        vx_act, _ = get_body_vel()
        r, p, _   = get_rpy_deg()
        err       = DESIRED_VEL_MPS - vx_act
        rospy.loginfo(f"  {elapsed:6.1f}s  {DESIRED_VEL_MPS:6.2f}  "
                      f"{vx_act:7.3f}  {err:+7.3f}  "
                      f"{get_alt():5.2f}m  {r:+6.1f}°  {p:+6.1f}°")
        last_log = now

    if elapsed >= FLY_DURATION_S:
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 6: 減速停止（線性爬坡 0→0，STOP_RAMP_S 秒）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Decelerating to 0 over {STOP_RAMP_S:.1f} s...")
t_stop = rospy.Time.now()
while not rospy.is_shutdown():
    elapsed = (rospy.Time.now() - t_stop).to_sec()
    if elapsed >= STOP_RAMP_S:
        break
    frac = elapsed / STOP_RAMP_S
    cmd_vel = DESIRED_VEL_MPS * (1.0 - frac)
    set_sp(make_vel_sp(vx_body=cmd_vel))
    rate.sleep()

# 速度歸零，切回位置懸停
cx, cy, cz = get_xyz()
set_sp(make_pos_sp(cx, cy, alt=cz))
rospy.loginfo(f"  Stopped at ({cx:.2f}, {cy:.2f}) m")

# 最終位移統計
rospy.loginfo(f"\n  ─── Flight summary ───────────────────────────")
rospy.loginfo(f"  Desired  : {DESIRED_VEL_MPS:.2f} m/s × {FLY_DURATION_S:.1f} s = "
              f"{DESIRED_VEL_MPS * FLY_DURATION_S:.2f} m")
rospy.loginfo(f"  Actual X : {cx:.2f} m (ENU, from takeoff origin)")
rospy.loginfo(f"  ──────────────────────────────────────────────")

# ─────────────────────────────────────────────────────────────
# Step 7: 懸停 3 s 後降落
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 7] Hover 3 s then AUTO.LAND...")
rospy.sleep(3.0)

_sp_timer.shutdown()
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
