#!/usr/bin/env python3
"""
Attitude Setpoint Hover Test (MAVROS / ROS 1)
=============================================
室內無 GPS 測試腳本：以 AttitudeTarget 發布姿態 + 推力 setpoint。

■ 與 hover_test_mavros.py 的差別
  hover_test_mavros.py  →  setpoint_position/local（需要 EKF position 估計，需 GPS/光流）
  本腳本               →  setpoint_raw/attitude  （只需 Yaw 估計，無 GPS 也能用）

■ 前提條件
  - 磁力計已校準（EKF Yaw estimate 有效）
  - HOVER_THRUST 已在 SITL 或低高度測試中確認（預設 0.55，需依機體調整）

■ 飛行序列
  1. 串流 attitude setpoint（thrust=0）→ OFFBOARD 前置條件
  2. 切換 OFFBOARD 模式
  3. 解鎖
  4. 推力線性爬升至 HOVER_THRUST（RAMP_DUR 秒）
  5. 水平姿態懸停 HOVER_DUR_S 秒
  6. AUTO.LAND 降落

⚠ 警告：attitude setpoint 沒有高度回授！
  HOVER_THRUST 太小 → 不起飛；太大 → 持續上升。
  第一次飛前手持機體在低空測試推力，確認能懸停後再放到地面執行。

執行方式：
  python3 att_hover_test_mavros.py
"""

import sys
import math
import threading
import rospy
from tf.transformations import quaternion_from_euler

from mavros_msgs.msg import State, AttitudeTarget, ParamValue
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
HOVER_THRUST  = 0.55    # 懸停推力 [0.0~1.0]，⚠ 需依機體實際調整
RAMP_DUR      = 5.0     # 推力從 0 爬升至 HOVER_THRUST 的時間 (s)
HOVER_DUR_S   = 15.0    # 懸停持續時間 (s)
YAW_DEG       = 0.0     # 目標偏航角（度，ENU: 0°=東）
CTRL_HZ       = 50      # 控制頻率（attitude 建議 ≥ 50 Hz）

# AttitudeTarget type_mask：忽略 body rate，僅傳送 orientation + thrust
# 數值來自 mavros_msgs/AttitudeTarget.msg 常數定義
MASK_IGNORE_RATES = 0b00000111   # = IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE

# ─────────────────────────────────────────────────────────────
# Shared state
# ─────────────────────────────────────────────────────────────
_state = State()
def _cb_state(msg): global _state; _state = msg

# 共享姿態 setpoint（主執行緒寫，Timer 執行緒讀）
_att = {
    'roll':   0.0,
    'pitch':  0.0,
    'yaw':    math.radians(YAW_DEG),
    'thrust': 0.0,
}
_att_lock = threading.Lock()

def update_att(**kwargs):
    """執行緒安全地更新姿態 setpoint 欄位。"""
    with _att_lock:
        for k, v in kwargs.items():
            if k in _att:
                _att[k] = float(v)

def _make_att_msg():
    with _att_lock:
        roll, pitch, yaw, thrust = (
            _att['roll'], _att['pitch'], _att['yaw'], _att['thrust']
        )
    msg = AttitudeTarget()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = 'base_link'
    msg.type_mask       = MASK_IGNORE_RATES
    q = quaternion_from_euler(roll, pitch, yaw)
    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]
    msg.thrust = thrust
    return msg

def _timer_publish(_event):
    att_pub.publish(_make_att_msg())

# ─────────────────────────────────────────────────────────────
# Service helpers
# ─────────────────────────────────────────────────────────────
def param_set(name, value):
    try:
        rospy.wait_for_service('/mavros/param/set', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        return svc(req).success
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
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('att_hover_test', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state', State, _cb_state)
att_pub = rospy.Publisher(
    '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

# Timer 在背景執行緒持續發布，確保 arm()/set_mode() 阻塞期間不中斷
_att_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_publish)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected!  mode={_state.mode}  armed={_state.armed}")
rospy.loginfo(f"  Config: HOVER_THRUST={HOVER_THRUST:.3f}, "
              f"RAMP={RAMP_DUR:.1f}s, HOVER={HOVER_DUR_S:.0f}s")

# ─────────────────────────────────────────────────────────────
# Step 0: PFA 姿態參數歸零（TVMD 自定義固件）
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
rospy.loginfo("  PFA_DES_ROLL  ... " + ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH ... " + ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1: 串流 attitude setpoint（thrust=0）作為 OFFBOARD 前置條件
# PX4 要求在切換 OFFBOARD 前已有 >2 Hz 的 setpoint 串流
# Timer 已在背景發布，主執行緒只需等待
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Pre-streaming attitude setpoints (5 s, thrust=0)...")
rospy.loginfo("  ※ 此步驟確認 PX4 接收到 setpoint 串流，解鎖前推力為 0")
rospy.sleep(5.0)

# ─────────────────────────────────────────────────────────────
# Step 2: 切換 OFFBOARD 模式
# 注意：仍需要 EKF Yaw estimate 有效（磁力計正常）
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 2] Requesting OFFBOARD mode...")
set_mode('OFFBOARD')

t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  OFFBOARD not confirmed after 5 s.")
        rospy.logwarn("  → 確認 EKF Yaw estimate 是否有效（QGC Messages 查看）")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_state.mode}")

if _state.mode != 'OFFBOARD':
    rospy.logerr("  Failed to enter OFFBOARD. Exiting.")
    _att_timer.shutdown()
    sys.exit(1)

# ─────────────────────────────────────────────────────────────
# Step 3: 解鎖
# Timer 在背景持續發布 attitude setpoint，PX4 不會因 COM_OF_LOSS_T 超時
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 3] Arming...")
arm()

t0 = rospy.Time.now()
while not rospy.is_shutdown() and not _state.armed:
    if (rospy.Time.now() - t0).to_sec() > 10.0:
        rospy.logerr("  ✗ Failed to arm. Check QGC for pre-arm errors.")
        _att_timer.shutdown()
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  ✓ Armed!")

# ─────────────────────────────────────────────────────────────
# Step 4: 線性爬升推力 0 → HOVER_THRUST（RAMP_DUR 秒）
# 目的：讓飛手有時間觀察馬達響應，必要時奪取遙控器控制
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Thrust ramp: 0 → {HOVER_THRUST:.3f} over {RAMP_DUR:.1f} s")
rospy.loginfo("  ⚠ 準備好隨時用遙控器切換模式（STABILIZED）奪回控制！")

t0 = rospy.Time.now()
while not rospy.is_shutdown():
    elapsed = (rospy.Time.now() - t0).to_sec()
    if elapsed >= RAMP_DUR:
        break
    t_now = HOVER_THRUST * (elapsed / RAMP_DUR)
    update_att(thrust=t_now)
    rate.sleep()

update_att(thrust=HOVER_THRUST)
rospy.loginfo(f"  Thrust reached: {HOVER_THRUST:.3f}")

# ─────────────────────────────────────────────────────────────
# Step 5: 水平姿態懸停 HOVER_DUR_S 秒
# roll=0, pitch=0, yaw=YAW_DEG, thrust=HOVER_THRUST
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Hovering {HOVER_DUR_S:.0f} s "
              f"(roll=0, pitch=0, yaw={YAW_DEG:.0f}°, thrust={HOVER_THRUST:.3f})")
rospy.loginfo(f"  {'Time':>6}  {'Mode':>10}  {'Thrust':>8}")
rospy.loginfo("  " + "─" * 28)

t0       = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t0).to_sec()

    if (now - last_log).to_sec() >= 2.0:
        rospy.loginfo(f"  {elapsed:6.1f}s  {_state.mode:>10}  {HOVER_THRUST:8.3f}")
        last_log = now

    if elapsed >= HOVER_DUR_S:
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 6: 降落
# AUTO.LAND 依氣壓計估計高度，無 GPS 室內仍可用
# ─────────────────────────────────────────────────────────────
_att_timer.shutdown()
rospy.loginfo("\n[Step 6] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
