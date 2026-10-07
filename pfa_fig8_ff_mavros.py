#!/usr/bin/env python3
"""
PFA Figure-8 Path Tracking with Feedforward (MAVROS / ROS1 Noetic)
===================================================================
Task: 繞 8 字軌跡飛行，roll=pitch=0°，yaw 跟隨前進切線方向
      pfa_fig8_mavros.py 的前饋版本：同時送出位置 + 速度 + 加速度

■ 為什麼要前饋
  pfa_fig8_mavros.py 只送位置（/mavros/setpoint_position/local），
  trajectory_setpoint 的 velocity / acceleration 為 NaN → pfa_pos_control 的
  _checkAllFinite() 把它們清成 0：

      control_acc = a_ref + Kv·(v_ref − v) + Kp·(p_ref − p)
                    = 0    + Kv·(0 − v)     + Kp·e

  等速前進時 Kp·e = Kv·v → 固定落後 e = (Kv/Kp)·v
  （exp_fig8_1007：Kp=25, Kv=20 → 0.8 s × 速度，實測沿前進方向誤差佔總誤差 68%）

  本腳本改用 /mavros/setpoint_raw/local (PositionTarget)，送出解析微分的
  v_ref = ṗ_d(t)、a_ref = p̈_d(t)，誤差動態變為 ë + Kv·ė + Kp·e = 0 → 收斂到 0。
  注意：前饋無法消除固定干擾（重心偏移、推力偏差）造成的穩態誤差，
        那需要 pfa_pos_control 的積分項。

■ 軌跡 (ENU，以懸停位置為 8 字中心)
  Lissajous 1:2 參數曲線，相位 θ(t)：
    x = cx + A·sin θ                 A = FIG8_LONG_RADIUS
    y = cy + B·sin 2θ                B = A/2
    z = TARGET_ALT_M
  θ(t)：角頻率 ω = 2π / PERIOD_S，前後各 RAMP_S 秒線性加減速
        （起停時速度連續，結束時回到中心且速度為 0）
  解析微分（θ̇、θ̈ 由 θ(t) 給出）：
    ẋ = A·cos θ·θ̇                    ẍ = −A·sin θ·θ̇² + A·cos θ·θ̈
    ẏ = 2B·cos 2θ·θ̇                  ÿ = −4B·sin 2θ·θ̇² + 2B·cos 2θ·θ̈
  yaw = atan2(2B·cos 2θ, A·cos θ)    （切線方向，與 θ̇ 大小無關，起停時不跳）

■ 限制
  pfa_pos_control 會把速度參考限制在 _speed_xy_max = 1.0 m/s
  （pfa_pos_control.hpp）；中心交叉速度 A·ω·√2 超過時前饋會被截斷，啟動時會警告。

■ 控制鏈
  set_sp() 更新共享 PositionTarget
    → rospy.Timer (CTRL_HZ, 背景執行緒)
    → /mavros/setpoint_raw/local（MAVROS 轉 ENU→NED）
    → SET_POSITION_TARGET_LOCAL_NED (position + velocity + acceleration + yaw)
    → trajectory_setpoint → pfa_pos_control → pfa_att_control

執行方式：
  python3 pfa_fig8_ff_mavros.py  (ROS 環境已 source)
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
)

# ─────────────────────────────────────────────────────────────
# Configuration（預設與 exp_fig8_1007 實驗相同，方便比較有無前饋）
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M        = 1.0    # 飛行高度 (m，ENU z+)
FIG8_LONG_RADIUS    = 1.0    # 8字長軸半徑 A (m); 短軸 B = A/2 自動計算
PERIOD_S            = 20.0   # 一個完整 8 字的時間 (s，等速段)
NUM_LAPS            = 2      # 飛行完整 8 字圈數
RAMP_S              = 3.0    # 起始加速 / 結束減速時間 (s)
YAW_TRACK_PATH      = True   # True: yaw 跟切線方向; False: 固定 yaw=0° (ENU East)

HOVER_PHASE_DUR     = 8.0    # 起飛後定高懸停時間 (s)
ALT_TOL             = 0.15   # 穩定懸停高度容差 (m)
HOVER_STABLE_TIME   = 3.0    # 穩定判定時間 (s)
CTRL_HZ             = 20     # Timer 發布頻率（經 SiK 數傳時建議降到 10）

PX4_SPEED_XY_MAX    = 1.0    # pfa_pos_control 的 _speed_xy_max (m/s)，速度參考上限

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
FIG8_SHORT_RADIUS = FIG8_LONG_RADIUS / 2.0       # B = A/2
OMEGA             = 2.0 * math.pi / PERIOD_S     # rad/s
THETA_END         = NUM_LAPS * 2.0 * math.pi
TOTAL_DUR_S       = THETA_END / OMEGA + RAMP_S   # 含加減速的總時間

# 中心交叉時速度最大：|v| = ω·sqrt(A² + (2B)²) = A·ω·√2 (B=A/2)
V_CENTER_MPS = OMEGA * math.sqrt(FIG8_LONG_RADIUS**2 + (2 * FIG8_SHORT_RADIUS)**2)
# 左右兩端 (x=±A) 速度與向心加速度：v = 2B·ω，a = 4B·ω²（ÿ 分量為 0，ẍ = −A·ω²）
V_LOBE_MPS = 2.0 * FIG8_SHORT_RADIUS * OMEGA
A_LOBE_MPS2 = FIG8_LONG_RADIUS * OMEGA ** 2

# PositionTarget.type_mask：位置、速度、加速度、yaw 都使用；只忽略 yaw_rate
# （pfa_pos_control / pfa_att_control 目前不讀 trajectory_setpoint.yawspeed）
TYPE_MASK_PVA_YAW = PositionTarget.IGNORE_YAW_RATE

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

def make_target(p, v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0), yaw_rad=0.0):
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
# Figure-8 trajectory (analytic position / velocity / acceleration)
# ─────────────────────────────────────────────────────────────

def fig8_phase(t):
    """回傳 (θ, θ̇, θ̈)：前後 RAMP_S 秒線性加減速，中段等角速度 ω。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= TOTAL_DUR_S:
        return THETA_END, 0.0, 0.0
    if t < RAMP_S:
        return 0.5 * OMEGA * t * t / RAMP_S, OMEGA * t / RAMP_S, OMEGA / RAMP_S
    if t > TOTAL_DUR_S - RAMP_S:
        r = TOTAL_DUR_S - t
        return THETA_END - 0.5 * OMEGA * r * r / RAMP_S, OMEGA * r / RAMP_S, -OMEGA / RAMP_S
    return 0.5 * OMEGA * RAMP_S + OMEGA * (t - RAMP_S), OMEGA, 0.0


def fig8_state(cx, cy, t):
    """t 秒時的 (p, v, a, yaw)，ENU，以 (cx, cy) 為中心。"""
    A, B = FIG8_LONG_RADIUS, FIG8_SHORT_RADIUS
    th, thd, thdd = fig8_phase(t)
    s1, c1 = math.sin(th),       math.cos(th)
    s2, c2 = math.sin(2.0 * th), math.cos(2.0 * th)

    p = (cx + A * s1, cy + B * s2, TARGET_ALT_M)
    v = (A * c1 * thd, 2.0 * B * c2 * thd, 0.0)
    a = (-A * s1 * thd * thd + A * c1 * thdd,
         -4.0 * B * s2 * thd * thd + 2.0 * B * c2 * thdd,
         0.0)
    # 切線方向只取決於 dx/dθ、dy/dθ（θ̇ ≥ 0），起停時 yaw 不跳
    yaw = math.atan2(2.0 * B * c2, A * c1) if YAW_TRACK_PATH else 0.0
    return p, v, a, yaw


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
rospy.init_node('pfa_fig8_ff', anonymous=False)
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
rospy.loginfo(f"  Figure-8 (feedforward) mission plan:")
rospy.loginfo(f"    Alt           : {TARGET_ALT_M:.1f} m (constant)")
rospy.loginfo(f"    Long radius A : {FIG8_LONG_RADIUS:.2f} m   Short radius B: {FIG8_SHORT_RADIUS:.2f} m")
rospy.loginfo(f"    Period        : {PERIOD_S:.0f} s/lap × {NUM_LAPS} laps "
              f"(+{RAMP_S:.0f} s ramp) = {TOTAL_DUR_S:.0f} s")
rospy.loginfo(f"    Speed         : center {V_CENTER_MPS:.2f} m/s, lobe tips {V_LOBE_MPS:.2f} m/s")
rospy.loginfo(f"    Accel (tips)  : {A_LOBE_MPS2:.2f} m/s²")
rospy.loginfo(f"    Feedforward   : position + velocity + acceleration (setpoint_raw/local)")
rospy.loginfo(f"    Yaw mode      : "
              f"{'追蹤切線方向' if YAW_TRACK_PATH else '固定 0° (ENU East)'}")
if V_CENTER_MPS > PX4_SPEED_XY_MAX:
    rospy.logwarn(f"  WARNING: peak speed {V_CENTER_MPS:.2f} m/s > pfa_pos_control _speed_xy_max "
                  f"{PX4_SPEED_XY_MAX:.1f} m/s → velocity feedforward will be clipped. "
                  f"Increase PERIOD_S or reduce FIG8_LONG_RADIUS.")

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
# Step 3 – Level hover hold, record figure-8 center, turn to initial yaw
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 3] Hover hold for {HOVER_PHASE_DUR:.0f} s (recording 8-center)...")
t_end    = rospy.Time.now() + rospy.Duration(HOVER_PHASE_DUR)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    now = rospy.Time.now()
    if (now - last_log).to_sec() > 2.0:
        rospy.loginfo(f"  alt={get_altitude():.2f} m  "
                      f"remaining={(t_end - now).to_sec():.1f} s")
        last_log = now
    rate.sleep()

cx, cy, _ = get_xyz()
rospy.loginfo(f"  Figure-8 center: ENU ({cx:.2f}, {cy:.2f}) m")

# 在中心先轉到起始切線方向，避免起步時 yaw 指令階躍
p0, _, _, yaw0 = fig8_state(cx, cy, 0.0)
YAW_SETTLE_S = 2.0
rospy.loginfo(f"  Turning to initial yaw {math.degrees(yaw0):.1f}° ({YAW_SETTLE_S:.0f} s)...")
set_sp(make_target(p0, yaw_rad=yaw0))
rospy.sleep(YAW_SETTLE_S)

# ─────────────────────────────────────────────────────────────
# Step 4 – Figure-8 trajectory with velocity / acceleration feedforward
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Figure-8 (FF): {NUM_LAPS} laps, {TOTAL_DUR_S:.0f} s total")
rospy.loginfo(f"  {'Time':>6}  {'Lap':>4}  {'Xcmd':>7}  {'Ycmd':>7}  {'Xnow':>7}  {'Ynow':>7}  "
              f"{'Err':>5}  {'|Vff|':>6}  {'|Aff|':>6}  {'Alt':>5}  {'YawCmd':>7}")
rospy.loginfo("  " + "─" * 88)

t_track  = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_track).to_sec()

    p, v, a, yaw_cmd = fig8_state(cx, cy, elapsed)
    set_sp(make_target(p, v, a, yaw_rad=yaw_cmd))

    if (now - last_log).to_sec() >= 0.5:
        xn, yn, _ = get_xyz()
        th, _, _  = fig8_phase(elapsed)
        err       = math.hypot(p[0] - xn, p[1] - yn)
        rospy.loginfo(f"  {elapsed:6.1f}s  {th / (2.0 * math.pi):4.2f}  {p[0]:7.2f}m  {p[1]:7.2f}m  "
                      f"{xn:7.2f}m  {yn:7.2f}m  {err:5.2f}  "
                      f"{math.hypot(v[0], v[1]):5.2f}   {math.hypot(a[0], a[1]):5.2f}   "
                      f"{get_altitude():5.2f}m  {math.degrees(yaw_cmd):6.1f}°")
        last_log = now

    if elapsed >= TOTAL_DUR_S:
        break
    rate.sleep()

rospy.loginfo("\n  Figure-8 complete.")

# ─────────────────────────────────────────────────────────────
# Step 5 – Hold at center
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Holding at center ({cx:.2f}, {cy:.2f}) m for 5 s...")
set_sp(make_target((cx, cy, TARGET_ALT_M), yaw_rad=fig8_state(cx, cy, TOTAL_DUR_S)[3]))
t_end    = rospy.Time.now() + rospy.Duration(5.0)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
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
