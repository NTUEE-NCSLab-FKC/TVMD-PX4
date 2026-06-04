#!/usr/bin/env python3
"""
PFA Velocity Test (MAVROS / ROS 1)
===================================
透過 pfa_pos_control 發送速度命令，同時以 PFA_DES_PITCH/ROLL=0° 維持水平姿態。
這是 TVMD 全驅動特性的驗證：機體不傾斜也能產生水平推力位移。

■ 控制鏈
  Python
    → /mavros/setpoint_raw/local (PositionTarget, velocity mode)
    → MAVLink SET_POSITION_TARGET_LOCAL_NED
    → pfa_pos_control  ← 接收速度目標
         PFA_DES_PITCH = 0°  → attitude_setpoint.pitch = 0  (不傾斜)
         PFA_DES_ROLL  = 0°  → attitude_setpoint.roll  = 0  (不傾斜)
    → pfa_att_control  ← 執行水平姿態
    → 馬達（水平推力向量負責移動）

■ 與 att_yaw_move_test_mavros.py 的差別
  att_yaw_move_test → AttitudeTarget → pfa_att_control（傾斜移動，傳統多旋翼做法）
  本腳本            → PositionTarget → pfa_pos_control（速度命令，機體不傾斜，PFA 特色）

■ 速度方向（機體 FLU，需轉換至 ENU world frame）
  前進 (Forward) : vx_body > 0
  後退 (Backward): vx_body < 0
  左移 (Left)    : vy_body > 0
  右移 (Right)   : vy_body < 0

■ 前提條件
  - 有效的 EKF local position 估計（GPS / 光流 / VIO）
  - 磁力計已校準（EKF Yaw estimate 有效）
  - SITL 或戶外 GPS 環境適用

執行方式：
  python3 pfa_vel_test_mavros.py
"""

import sys
import math
import threading
import rospy
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, ParamValue
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M = 1.0     # 懸停高度 (m)
VEL_MPS      = 0.5     # 前後左右速度 (m/s)
VEL_DUR      = 4.0     # 每個方向速度命令持續時間 (s)
STOP_DUR     = 3.0     # 動作間停止時間 (s)
CTRL_HZ      = 20      # 控制頻率

# ─────────────────────────────────────────────────────────────
# PositionTarget type_mask 常數（數值避免 import 依賴）
#
# PX4 / MAVLink SET_POSITION_TARGET_LOCAL_NED type_mask：
#   bit 0  = IGNORE_PX (1)       bit 6  = IGNORE_AFX (64)
#   bit 1  = IGNORE_PY (2)       bit 7  = IGNORE_AFY (128)
#   bit 2  = IGNORE_PZ (4)       bit 8  = IGNORE_AFZ (256)
#   bit 3  = IGNORE_VX (8)       bit 10 = IGNORE_YAW (1024)
#   bit 4  = IGNORE_VY (16)      bit 11 = IGNORE_YAW_RATE (2048)
#   bit 5  = IGNORE_VZ (32)
#
# 位置模式：使用 pos + yaw，忽略 vel / accel / yaw_rate
MASK_POS = 8 + 16 + 32 + 64 + 128 + 256 + 2048        # = 2552
# 速度模式：使用 vel + yaw，忽略 pos / accel / yaw_rate
MASK_VEL = 1 + 2 + 4 + 64 + 128 + 256 + 2048          # = 2503
# ─────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────
# Subscribers
# ─────────────────────────────────────────────────────────────
_state = State()
_pose  = PoseStamped()
_imu   = Imu()

def _cb_state(msg): global _state; _state = msg
def _cb_pose(msg):  global _pose;  _pose  = msg
def _cb_imu(msg):   global _imu;   _imu   = msg

def get_pos():
    return _pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z

def get_yaw_rad():
    o = _imu.orientation
    _, _, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return y

def get_rpy_deg():
    o = _imu.orientation
    r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(r), math.degrees(p), math.degrees(y)

# ─────────────────────────────────────────────────────────────
# Shared PositionTarget setpoint（主執行緒寫，Timer 讀）
# ─────────────────────────────────────────────────────────────
_sp_lock = threading.Lock()
_sp_msg  = [None]

def _update_sp(msg):
    with _sp_lock:
        _sp_msg[0] = msg

def _timer_publish(_event):
    with _sp_lock:
        msg = _sp_msg[0]
    if msg is not None:
        msg.header.stamp = rospy.Time.now()
        sp_pub.publish(msg)

# ─────────────────────────────────────────────────────────────
# Setpoint builders
# ─────────────────────────────────────────────────────────────
def make_pos_sp(x, y, alt=TARGET_ALT_M, yaw=0.0):
    """位置控制：懸停在 (x, y, alt) + 指定 yaw（ENU frame）。"""
    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1        # FRAME_LOCAL_NED → MAVROS 轉成 ENU
    msg.type_mask        = MASK_POS
    msg.position.x       = x
    msg.position.y       = y
    msg.position.z       = alt
    msg.yaw              = yaw
    return msg

def make_vel_sp(vx_body=0.0, vy_body=0.0, vz=0.0):
    """
    速度控制（ENU world frame）。
    機體 FLU 速度轉換至 ENU：
      vx_enu =  vx_body * cos(yaw) - vy_body * sin(yaw)
      vy_enu =  vx_body * sin(yaw) + vy_body * cos(yaw)
    維持當前 yaw（不旋轉）。
    """
    yaw    = get_yaw_rad()
    vx_enu = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
    vy_enu = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)

    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1
    msg.type_mask        = MASK_VEL
    msg.velocity.x       = vx_enu
    msg.velocity.y       = vy_enu
    msg.velocity.z       = vz
    msg.yaw              = yaw      # 維持 heading，不旋轉
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
            if svc(req).success:
                return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"param_set({name}): {e}")
    return False

def set_mode(mode):
    try:
        rospy.wait_for_service('/mavros/set_mode', timeout=5)
        svc = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        return svc(SetModeRequest(custom_mode=mode)).mode_sent
    except Exception as e:
        rospy.logwarn(f"set_mode({mode}): {e}")
        return False

def arm():
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        return svc(CommandBoolRequest(value=True)).success
    except Exception as e:
        rospy.logwarn(f"arming: {e}")
        return False

# ─────────────────────────────────────────────────────────────
# Phase helpers
# ─────────────────────────────────────────────────────────────
def hover_phase(dur, x=None, y=None, label=''):
    """
    位置控制懸停 dur 秒。
    x/y 未指定則懸停在當前位置（速度命令結束後使用）。
    """
    cx, cy, cz = get_pos()
    sp = make_pos_sp(x if x is not None else cx,
                     y if y is not None else cy,
                     alt=TARGET_ALT_M)
    _update_sp(sp)
    if label:
        rospy.loginfo(f"  [hover] {label}  ({dur:.1f} s)")
    t0 = last_log = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if (now - last_log).to_sec() >= 2.0:
            r, p, y = get_rpy_deg()
            cx, cy, cz = get_pos()
            rospy.loginfo(f"    pos=({cx:.2f},{cy:.2f},{cz:.2f})m  "
                          f"roll={r:+.1f}°  pitch={p:+.1f}°  yaw={y:+.1f}°")
            last_log = now
        if (now - t0).to_sec() >= dur:
            break
        rate.sleep()


def vel_phase(vx_body, vy_body=0.0, label=''):
    """
    速度命令 VEL_DUR 秒（pfa_pos_control），每秒印一次姿態確認不傾斜。
    結束後切回位置懸停。
    """
    rospy.loginfo(f"  [vel] {label}  "
                  f"vx_body={vx_body:+.2f}  vy_body={vy_body:+.2f} m/s  ({VEL_DUR:.1f} s)")
    t0 = last_log = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        _update_sp(make_vel_sp(vx_body, vy_body))   # 每幀更新以使用最新 yaw
        if (now - last_log).to_sec() >= 1.0:
            r, p, y = get_rpy_deg()
            cx, cy, cz = get_pos()
            rospy.loginfo(f"    pos=({cx:.2f},{cy:.2f},{cz:.2f})m  "
                          f"roll={r:+.1f}°  pitch={p:+.1f}°  yaw={y:+.1f}°")
            last_log = now
        if (now - t0).to_sec() >= VEL_DUR:
            break
        rate.sleep()
    # 速度歸零：切回位置模式懸停在現位
    _update_sp(make_pos_sp(*get_pos()[:2], alt=TARGET_ALT_M))

# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('pfa_vel_test', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)
sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

_update_sp(make_pos_sp(0.0, 0.0))   # 初始 setpoint（position mode）
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_publish)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected!  mode={_state.mode}  armed={_state.armed}")
rospy.loginfo(f"  Config: alt={TARGET_ALT_M}m  vel={VEL_MPS}m/s  dur={VEL_DUR}s")

# ─────────────────────────────────────────────────────────────
# Step 0: PFA 姿態參數歸零（確保 pfa_pos_control 輸出水平姿態）
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] PFA_DES_ROLL=0°, PFA_DES_PITCH=0°  (level attitude for PFA)")
rospy.loginfo("  PFA_DES_ROLL  ... " + ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH ... " + ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1-3: Pre-stream → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 1] Pre-streaming position setpoints (5 s)...")
rospy.sleep(5.0)

rospy.loginfo("\n[Step 2] Requesting OFFBOARD mode...")
set_mode('OFFBOARD')
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logerr("  OFFBOARD timeout. Exiting.")
        _sp_timer.shutdown(); sys.exit(1)
    rate.sleep()
rospy.loginfo(f"  Mode: {_state.mode}")

rospy.loginfo("\n[Step 3] Arming...")
arm()
t0 = rospy.Time.now()
while not rospy.is_shutdown() and not _state.armed:
    if (rospy.Time.now() - t0).to_sec() > 10.0:
        rospy.logerr("  Arm timeout. Exiting.")
        _sp_timer.shutdown(); sys.exit(1)
    rate.sleep()
rospy.loginfo("  ✓ Armed!")

# ─────────────────────────────────────────────────────────────
# Step 4: 起飛至 TARGET_ALT_M（位置控制）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Takeoff & stabilize at {TARGET_ALT_M} m (position control)...")
hover_phase(5.0, x=0.0, y=0.0, label=f'起飛 @ {TARGET_ALT_M} m')

# ─────────────────────────────────────────────────────────────
# Step 5: 速度測試
#
# 透過 pfa_pos_control 接收速度目標：
#   VEL 命令 → pos_ctrl 計算推力向量
#   PFA_DES_PITCH=0 → 機體姿態保持水平
#   → 不同於傳統多旋翼（傾斜才能移動），PFA 水平推力直接產生位移
#
# 觀察重點：roll/pitch 應維持在 ≈0°（TVMD PFA 特性）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] PFA Velocity test  v={VEL_MPS:.2f} m/s  dur={VEL_DUR:.1f} s")
rospy.loginfo("  觀察：roll/pitch 應保持 ≈0°（PFA 水平推力，機體不傾斜）")
rospy.loginfo("  對比：傳統多旋翼前進時 pitch ≈ -8~-15°")

vel_phase(+VEL_MPS,  0.0,      '前進 Forward  (vx_body=+)')
hover_phase(STOP_DUR, label='停止 Stop')

vel_phase(-VEL_MPS,  0.0,      '後退 Backward (vx_body=-)')
hover_phase(STOP_DUR, label='停止 Stop')

vel_phase( 0.0,     +VEL_MPS,  '左移 Left     (vy_body=+)')
hover_phase(STOP_DUR, label='停止 Stop')

vel_phase( 0.0,     -VEL_MPS,  '右移 Right    (vy_body=-)')
hover_phase(STOP_DUR, label='停止 Stop')

hover_phase(2.0, label='動作完成')

# ─────────────────────────────────────────────────────────────
# Step 6: 降落
# ─────────────────────────────────────────────────────────────
_sp_timer.shutdown()
rospy.loginfo("\n[Step 6] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
