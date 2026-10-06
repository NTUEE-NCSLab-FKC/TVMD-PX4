#!/usr/bin/env python3
"""
Circle Trajectory (MAVROS / ROS 1)
====================================
光流室內版：繞直徑 1 m（半徑 0.5 m）圓形軌跡飛行。
架構與 square_traj_mavros.py / square_traj_cw_mavros.py 相同。

■ 控制鏈
  set_sp() 更新共享 msg
    → rospy.Timer (CTRL_HZ, 背景執行緒)
    → /mavros/setpoint_position/local
    → pfa_pos_control  (PFA_DES_PITCH=0, PFA_DES_ROLL=0)
    → pfa_att_control
    → 馬達

■ 圓形軌跡（以懸停位置為中心，ENU）
  起點 = 圓的東側 (cx+R, cy)，θ=0
  位置：x = cx + R·cos(dir·θ),  y = cy + R·sin(dir·θ)
        dir = +1 逆時針 (CCW)、-1 順時針 (CW)，由 CLOCKWISE 切換（由上往下看）
  θ(t)：角速度 ω = SPEED_MPS / R，前後各 RAMP_S 秒線性加減速，
        避免起停時速度指令階躍
  yaw ：YAW_TRACK_PATH=True 時追蹤切線方向（dir·θ + dir·90°）

■ 降落
  確認進入 AUTO.LAND 後才停止 Timer，避免切換途中先觸發 offboard lost

執行方式：
  python3 circle_traj_mavros.py
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
TARGET_ALT_M      = 0.8    # 懸停高度 (m)，光流室內建議 0.5~1.0 m
CIRCLE_DIAMETER_M = 1.0    # 圓直徑 (m)
SPEED_MPS         = 0.3    # 切線速度 (m/s)
RAMP_S            = 3.0    # 起始加速 / 結束減速時間 (s)
NUM_LAPS          = 2      # 完整圓圈數
CLOCKWISE         = False  # False: 逆時針 (CCW); True: 順時針 (CW)，由上往下看
YAW_TRACK_PATH    = True   # True: yaw 追蹤前進方向; False: 固定 yaw=0°

HOVER_PHASE_DUR   = 5.0    # 起飛後穩定等待時間 (s)
ALT_TOL           = 0.10   # 高度穩定容忍 (m)
HOVER_STABLE_TIME = 3.0    # 連續穩定秒數
CTRL_HZ           = 20     # Timer 發布頻率

# Derived
_R         = CIRCLE_DIAMETER_M / 2.0                       # 半徑 (m)
_OMEGA     = SPEED_MPS / _R                                # 角速度 (rad/s)
_DIR       = -1.0 if CLOCKWISE else 1.0
_THETA_END = NUM_LAPS * 2.0 * math.pi                      # 總角度 (rad)
_TOTAL_S   = _THETA_END / _OMEGA + RAMP_S                  # 含加減速的總時間 (s)
_LAP_DUR_S = 2.0 * math.pi / _OMEGA                        # 等速時每圈時間 (s)
_ACC_C     = SPEED_MPS ** 2 / _R                           # 向心加速度 (m/s²)


def circle_theta(t):
    """t 秒時的角度 θ (rad, ≥0)：前後 RAMP_S 秒線性加減速，中段等角速度。"""
    if t <= 0.0:
        return 0.0
    if t >= _TOTAL_S:
        return _THETA_END
    if t < RAMP_S:
        return 0.5 * _OMEGA * t * t / RAMP_S
    if t > _TOTAL_S - RAMP_S:
        return _THETA_END - 0.5 * _OMEGA * (_TOTAL_S - t) ** 2 / RAMP_S
    return 0.5 * _OMEGA * RAMP_S + _OMEGA * (t - RAMP_S)


def circle_point(cx, cy, theta):
    """θ 對應的位置與切線 yaw（ENU）。"""
    a = _DIR * theta
    x = cx + _R * math.cos(a)
    y = cy + _R * math.sin(a)
    yaw = (a + _DIR * math.pi / 2.0) if YAW_TRACK_PATH else 0.0
    return x, y, yaw

# ─────────────────────────────────────────────────────────────
# Subscribers
# ─────────────────────────────────────────────────────────────
_vehicle_state = State()
_local_pose    = PoseStamped()
_imu_data      = Imu()

def _cb_state(msg): global _vehicle_state; _vehicle_state = msg
def _cb_pose(msg):  global _local_pose;    _local_pose    = msg
def _cb_imu(msg):   global _imu_data;      _imu_data      = msg

def get_altitude(): return _local_pose.pose.position.z
def get_xyz():
    p = _local_pose.pose.position; return p.x, p.y, p.z
def get_rpy_deg():
    o = _imu_data.orientation
    r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(r), math.degrees(p), math.degrees(y)

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
    """20 Hz 背景回呼：更新 timestamp 後發布。"""
    with _sp_lock:
        msg = _sp_msg[0]
    if msg is not None:
        msg.header.stamp = rospy.Time.now()
        sp_pub.publish(msg)

def make_sp(x, y, z=TARGET_ALT_M, yaw_rad=0.0):
    """建立 ENU PoseStamped（orientation 含 yaw，防止 pfa_att_control NaN）。"""
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
rospy.init_node('circle_traj', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)
sp_pub = rospy.Publisher(
    '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

set_sp(make_sp(0.0, 0.0))   # 初始 setpoint（Timer 啟動前先設定）
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_cb)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _vehicle_state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected.  mode={_vehicle_state.mode}  armed={_vehicle_state.armed}")
rospy.loginfo(f"  Plan: alt={TARGET_ALT_M}m  diameter={CIRCLE_DIAMETER_M}m  "
              f"speed={SPEED_MPS}m/s  {'CW' if CLOCKWISE else 'CCW'}  "
              f"~{_LAP_DUR_S:.1f}s/lap × {NUM_LAPS} laps (+{RAMP_S:.0f}s ramp)  "
              f"a_c={_ACC_C:.2f}m/s²")

# ─────────────────────────────────────────────────────────────
# Step 0: PFA 參數歸零
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
rospy.loginfo("  PFA_DES_ROLL  ... " + ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH ... " + ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1: Pre-stream 5 s（OFFBOARD 前置條件）
# Timer 在背景發布，rospy.sleep() 讓主執行緒等待即可
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 1] Pre-streaming setpoints (5 s)...")
rospy.sleep(5.0)

# ─────────────────────────────────────────────────────────────
# Step 2: 切換 OFFBOARD
# arm()/set_mode() 阻塞時，Timer 仍在背景 20 Hz 發布
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 2] Requesting OFFBOARD mode...")
set_mode('OFFBOARD')
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _vehicle_state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_vehicle_state.mode}")

# ─────────────────────────────────────────────────────────────
# Step 3: 解鎖
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 3] Arming...")
arm_vehicle()
t0 = rospy.Time.now()
while not rospy.is_shutdown() and not _vehicle_state.armed:
    if (rospy.Time.now() - t0).to_sec() > 10.0:
        rospy.logerr("  ✗ Failed to arm. Exiting.")
        _sp_timer.shutdown(); sys.exit(1)
    rate.sleep()
rospy.loginfo("  ✓ Armed!")

# ─────────────────────────────────────────────────────────────
# Step 4: 起飛，等待高度穩定
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Takeoff to {TARGET_ALT_M} m...")
set_sp(make_sp(0.0, 0.0, TARGET_ALT_M))

stable_since = None
last_log     = rospy.Time.now()
t_phase      = rospy.Time.now()

while not rospy.is_shutdown():
    now, alt = rospy.Time.now(), get_altitude()
    alt_err  = TARGET_ALT_M - alt
    stable_since = (stable_since or now) if abs(alt_err) < ALT_TOL else None

    if (now - last_log).to_sec() >= 1.0:
        s = (now - stable_since).to_sec() if stable_since else 0.0
        rospy.loginfo(f"  alt={alt:.2f}m  err={alt_err:+.2f}m  "
                      f"stable={s:.1f}/{HOVER_STABLE_TIME:.0f}s")
        last_log = now
    if stable_since and (now - stable_since).to_sec() >= HOVER_STABLE_TIME:
        rospy.loginfo(f"  ✓ Stable at {alt:.2f} m.")
        break
    if (now - t_phase).to_sec() > 20.0:
        rospy.logwarn(f"  Hover timeout, alt={alt:.2f} m, continuing.")
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 5: 穩定懸停，記錄圓心座標
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Hover hold {HOVER_PHASE_DUR:.0f} s (記錄圓心)...")
t_end = rospy.Time.now() + rospy.Duration(HOVER_PHASE_DUR)
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    rate.sleep()

cx, cy, _ = get_xyz()
rospy.loginfo(f"  Circle centre: ({cx:.3f}, {cy:.3f}) m (ENU)")

# ─────────────────────────────────────────────────────────────
# Step 6: 由圓心移動至起點 (cx+R, cy)，並轉向起始切線方向
# ─────────────────────────────────────────────────────────────
APPROACH_DUR = 3.0
YAW_SETTLE_S = 2.0
p0x, p0y, init_yaw = circle_point(cx, cy, 0.0)

rospy.loginfo(f"\n[Step 6] Approach start point ({p0x:.2f},{p0y:.2f}) in {APPROACH_DUR:.0f} s, "
              f"yaw→{math.degrees(init_yaw):.0f}°...")
t_app = rospy.Time.now()
while not rospy.is_shutdown():
    elapsed = (rospy.Time.now() - t_app).to_sec()
    frac    = min(elapsed / APPROACH_DUR, 1.0)
    s       = frac * frac * (3.0 - 2.0 * frac)   # cubic ease-in-out
    set_sp(make_sp(cx + s * (p0x - cx),
                   cy + s * (p0y - cy),
                   TARGET_ALT_M, yaw_rad=init_yaw))
    if frac >= 1.0: break
    rate.sleep()
rospy.sleep(YAW_SETTLE_S)   # 起點停留，讓 yaw 轉到切線方向
rospy.loginfo(f"  At start point.")

# ─────────────────────────────────────────────────────────────
# Step 7: 圓形軌跡
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 7] Circle trajectory ({NUM_LAPS} lap(s), "
              f"{'CW' if CLOCKWISE else 'CCW'})  ~{_TOTAL_S:.0f} s total")

t_circ   = rospy.Time.now()
last_log = rospy.Time.now()
while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t_circ).to_sec()
    theta   = circle_theta(elapsed)
    x_cmd, y_cmd, yaw_cmd = circle_point(cx, cy, theta)
    set_sp(make_sp(x_cmd, y_cmd, TARGET_ALT_M, yaw_rad=yaw_cmd))

    if (now - last_log).to_sec() >= 1.0:
        xn, yn, _ = get_xyz()
        r, p, _ = get_rpy_deg()
        err = math.hypot(xn - x_cmd, yn - y_cmd)
        rospy.loginfo(f"    {elapsed:5.1f}s  lap={theta / (2.0 * math.pi):.2f}  "
                      f"cmd=({x_cmd:.2f},{y_cmd:.2f})  pos=({xn:.2f},{yn:.2f})  "
                      f"err={err:.2f}m  alt={get_altitude():.2f}m  "
                      f"yaw_cmd={math.degrees(yaw_cmd) % 360.0:5.1f}°  "
                      f"roll={r:+.1f}°  pitch={p:+.1f}°")
        last_log = now
    if elapsed >= _TOTAL_S: break
    rate.sleep()

rospy.loginfo(f"\n  ✓ Circle trajectory complete ({NUM_LAPS} lap(s)).")

# ─────────────────────────────────────────────────────────────
# Step 8: 返回中心懸停 5 s
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 8] Return to centre ({cx:.2f}, {cy:.2f}) m...")
RETURN_DUR = 3.0
rx0, ry0, _ = get_xyz()   # 捕捉起始位置後再插值
t_ret = rospy.Time.now()
while not rospy.is_shutdown():
    elapsed = (rospy.Time.now() - t_ret).to_sec()
    frac    = min(elapsed / RETURN_DUR, 1.0)
    s       = frac * frac * (3.0 - 2.0 * frac)
    set_sp(make_sp(rx0 + s * (cx - rx0),
                   ry0 + s * (cy - ry0),
                   TARGET_ALT_M, yaw_rad=0.0))
    if frac >= 1.0: break
    rate.sleep()

set_sp(make_sp(cx, cy, TARGET_ALT_M))
t_end    = rospy.Time.now() + rospy.Duration(5.0)
last_log = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    now = rospy.Time.now()
    if (now - last_log).to_sec() >= 2.0:
        xn, yn, _ = get_xyz()
        r, p, _ = get_rpy_deg()
        rospy.loginfo(f"  pos=({xn:.2f},{yn:.2f})  alt={get_altitude():.2f}m  "
                      f"roll={r:+.1f}°  pitch={p:+.1f}°")
        last_log = now
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 9: 降落
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 9] Landing (AUTO.LAND)...")
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
