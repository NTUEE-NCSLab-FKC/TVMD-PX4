#!/usr/bin/env python3
"""
Attitude Setpoint: Yaw & Translation Test (MAVROS / ROS 1)
==========================================================
建立在 att_hover_test_mavros.py 之上，測試 yaw 旋轉與四方向平移。

■ 飛行序列
  1. 起飛（推力爬升至 HOVER_THRUST）
  2. 穩定懸停 3 s
  3. Yaw 測試：+90° → 0° → -90° → 0°（逐步旋轉，不跳躍）
  4. 平移測試（傾斜機體）：前進 → 後退 → 左移 → 右移
  5. AUTO.LAND

■ 座標系（MAVROS ENU / 機體 FLU）
  前進  → pitch = -TILT_DEG（機頭下傾）
  後退  → pitch = +TILT_DEG（機頭上仰）
  左移  → roll  = -TILT_DEG（機體向左傾）
  右移  → roll  = +TILT_DEG（機體向右傾）
  yaw+  → 逆時針（從上往下看）

⚠ 無位置控制：平移段機體會持續漂移，確保每方向至少 3-5 m 淨空。

執行方式：
  python3 att_yaw_move_test_mavros.py
"""

import sys
import math
import threading
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
HOVER_THRUST  = 0.55    # 懸停推力 [0~1]，依 att_hover_test 校準結果填入
RAMP_DUR      = 5.0     # 推力爬升時間 (s)
CTRL_HZ       = 50      # 控制頻率

# Yaw 測試
YAW_RATE_DPS  = 30.0    # Yaw 旋轉速率 (度/秒)
YAW_TARGETS   = [90.0, 0.0, -90.0, 0.0]   # 依序旋轉到這些角度 (度)
YAW_DWELL_S   = 2.0     # 每個目標到達後停留時間 (s)

# 平移測試
TILT_DEG      = 8.0     # 傾斜角度 (度)，建議 5-10°
MOVE_DUR      = 3.0     # 每個方向的傾斜持續時間 (s)
DWELL_DUR     = 2.0     # 動作間的水平懸停時間 (s)

MASK_IGNORE_RATES = 0b00000111   # IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE

# ─────────────────────────────────────────────────────────────
# Subscribers
# ─────────────────────────────────────────────────────────────
_state = State()
_imu   = Imu()

def _cb_state(msg): global _state; _state = msg
def _cb_imu(msg):   global _imu;   _imu   = msg

def get_rpy_deg():
    """從 IMU 四元數取得當前 roll/pitch/yaw（度）。"""
    o = _imu.orientation
    r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(r), math.degrees(p), math.degrees(y)

# ─────────────────────────────────────────────────────────────
# Shared attitude setpoint（主執行緒寫，Timer 執行緒讀）
# ─────────────────────────────────────────────────────────────
_att      = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'thrust': 0.0}
_att_lock = threading.Lock()

def update_att(**kwargs):
    with _att_lock:
        for k, v in kwargs.items():
            if k in _att:
                _att[k] = float(v)

def _make_att_msg():
    with _att_lock:
        r, p, y, t = _att['roll'], _att['pitch'], _att['yaw'], _att['thrust']
    msg = AttitudeTarget()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = 'base_link'
    msg.type_mask       = MASK_IGNORE_RATES
    q = quaternion_from_euler(r, p, y)
    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]
    msg.thrust = t
    return msg

def _timer_publish(_event):
    att_pub.publish(_make_att_msg())

# ─────────────────────────────────────────────────────────────
# Movement helpers
# ─────────────────────────────────────────────────────────────
def hover_level(dur, label=''):
    """水平姿態（roll=0, pitch=0）懸停 dur 秒，每 2 s 印姿態。"""
    update_att(roll=0.0, pitch=0.0)
    if label:
        rospy.loginfo(f"  [hover] {label}  ({dur:.1f} s)")
    t0 = last_log = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if (now - last_log).to_sec() >= 2.0:
            r, p, y = get_rpy_deg()
            rospy.loginfo(f"    roll={r:+6.1f}°  pitch={p:+6.1f}°  yaw={y:+6.1f}°")
            last_log = now
        if (now - t0).to_sec() >= dur:
            break
        rate.sleep()


def yaw_to(target_deg):
    """
    以 YAW_RATE_DPS 速率線性旋轉 yaw 至 target_deg。
    使用命令 yaw（非 IMU 估計）插值，確保平滑旋轉。
    """
    with _att_lock:
        current_yaw_deg = math.degrees(_att['yaw'])

    diff = target_deg - current_yaw_deg
    # 角度差歸一化到 (-180, 180]
    diff = (diff + 180) % 360 - 180

    if abs(diff) < 0.5:
        return

    dur  = abs(diff) / YAW_RATE_DPS
    sign = math.copysign(1.0, diff)

    rospy.loginfo(f"  [yaw] {current_yaw_deg:+.1f}° → {target_deg:+.1f}°  "
                  f"(Δ={diff:+.1f}°, {dur:.1f} s @ {YAW_RATE_DPS:.0f} °/s)")

    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - t0).to_sec()
        if elapsed >= dur:
            break
        cmd_yaw = current_yaw_deg + sign * YAW_RATE_DPS * elapsed
        update_att(yaw=math.radians(cmd_yaw))
        rate.sleep()

    update_att(yaw=math.radians(target_deg))
    _, _, y_est = get_rpy_deg()
    rospy.loginfo(f"    ✓ arrived  cmd={target_deg:+.1f}°  IMU={y_est:+.1f}°")


def move(roll_deg=0.0, pitch_deg=0.0, label=''):
    """
    傾斜機體 MOVE_DUR 秒後歸零。
    roll_deg / pitch_deg 使用 ENU/FLU 符號：
      pitch=-  →  機頭下傾  →  前進
      pitch=+  →  機頭上仰  →  後退
      roll=-   →  向左傾    →  左移
      roll=+   →  向右傾    →  右移
    """
    rospy.loginfo(f"  [move] {label}  "
                  f"roll={roll_deg:+.1f}°  pitch={pitch_deg:+.1f}°  ({MOVE_DUR:.1f} s)")
    update_att(roll=math.radians(roll_deg), pitch=math.radians(pitch_deg))

    t0 = last_log = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if (now - last_log).to_sec() >= 1.0:
            r, p, y = get_rpy_deg()
            rospy.loginfo(f"    roll={r:+6.1f}°  pitch={p:+6.1f}°  yaw={y:+6.1f}°")
            last_log = now
        if (now - t0).to_sec() >= MOVE_DUR:
            break
        rate.sleep()

    update_att(roll=0.0, pitch=0.0)

# ─────────────────────────────────────────────────────────────
# Service helpers
# ─────────────────────────────────────────────────────────────
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
rospy.init_node('att_yaw_move_test', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',    State, _cb_state)
rospy.Subscriber('/mavros/imu/data', Imu,   _cb_imu)
att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

_att_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_publish)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected!  mode={_state.mode}  armed={_state.armed}")
rospy.loginfo(f"  Config: thrust={HOVER_THRUST}, yaw_rate={YAW_RATE_DPS}°/s, "
              f"tilt={TILT_DEG}°, move={MOVE_DUR}s")

# ─────────────────────────────────────────────────────────────
# Step 1-3: Pre-stream → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 1] Pre-streaming setpoints (5 s, thrust=0)...")
rospy.sleep(5.0)

rospy.loginfo("\n[Step 2] Requesting OFFBOARD mode...")
set_mode('OFFBOARD')
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logerr("  OFFBOARD timeout. Exiting.")
        _att_timer.shutdown(); sys.exit(1)
    rate.sleep()
rospy.loginfo(f"  Mode: {_state.mode}")

rospy.loginfo("\n[Step 3] Arming...")
arm()
t0 = rospy.Time.now()
while not rospy.is_shutdown() and not _state.armed:
    if (rospy.Time.now() - t0).to_sec() > 10.0:
        rospy.logerr("  Arm timeout. Exiting.")
        _att_timer.shutdown(); sys.exit(1)
    rate.sleep()
rospy.loginfo("  ✓ Armed!")

# ─────────────────────────────────────────────────────────────
# Step 4: Thrust ramp 0 → HOVER_THRUST
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Thrust ramp 0 → {HOVER_THRUST:.3f} over {RAMP_DUR:.1f} s")
rospy.loginfo("  ⚠ 準備好遙控器，隨時可切換 STABILIZED 奪回控制！")
t0 = rospy.Time.now()
while not rospy.is_shutdown():
    elapsed = (rospy.Time.now() - t0).to_sec()
    if elapsed >= RAMP_DUR:
        break
    update_att(thrust=HOVER_THRUST * elapsed / RAMP_DUR)
    rate.sleep()
update_att(thrust=HOVER_THRUST)
rospy.loginfo(f"  Thrust reached: {HOVER_THRUST:.3f}")

# ─────────────────────────────────────────────────────────────
# Step 5: 起飛穩定
# ─────────────────────────────────────────────────────────────
hover_level(3.0, '起飛穩定')

# ─────────────────────────────────────────────────────────────
# Step 6: Yaw 測試
# +90°（逆時針） → 0° → -90°（順時針） → 0°
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 6] Yaw test: {YAW_TARGETS}°  rate={YAW_RATE_DPS:.0f}°/s")
for target in YAW_TARGETS:
    yaw_to(target)
    hover_level(YAW_DWELL_S)

# ─────────────────────────────────────────────────────────────
# Step 7: 平移測試（傾斜機體）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 7] Translation test  tilt={TILT_DEG:.0f}°  dur={MOVE_DUR:.1f}s")
rospy.loginfo("  ENU/FLU convention:")
rospy.loginfo(f"    pitch=-{TILT_DEG:.0f}° → 前進 (Forward)")
rospy.loginfo(f"    pitch=+{TILT_DEG:.0f}° → 後退 (Backward)")
rospy.loginfo(f"    roll=-{TILT_DEG:.0f}°  → 左移 (Left)")
rospy.loginfo(f"    roll=+{TILT_DEG:.0f}°  → 右移 (Right)")

MOVES = [
    ('前進 Forward',  0.0,        -TILT_DEG),
    ('後退 Backward', 0.0,        +TILT_DEG),
    ('左移 Left',    -TILT_DEG,    0.0),
    ('右移 Right',   +TILT_DEG,    0.0),
]

for label, r_deg, p_deg in MOVES:
    hover_level(DWELL_DUR)
    move(roll_deg=r_deg, pitch_deg=p_deg, label=label)

hover_level(DWELL_DUR, '動作完成，準備降落')

# ─────────────────────────────────────────────────────────────
# Step 8: 降落
# ─────────────────────────────────────────────────────────────
_att_timer.shutdown()
rospy.loginfo("\n[Step 8] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
