#!/usr/bin/env python3
"""
Square Trajectory (MAVROS / ROS 1)
====================================
光流室內版：繞 1 m × 1 m 方形軌跡飛行。

■ 與 pfa_star_mavros.py 的修正
  原版：arm/set_mode 阻塞時 sp_pub 停止 → COM_OF_LOSS_T 可能超時
  本版：rospy.Timer 在背景執行緒持續以 CTRL_HZ 發布 setpoint，
        arm()/set_mode() 阻塞期間 PX4 不會因 offboard 訊號中斷而拒絕

■ 控制鏈
  set_sp() 更新共享 msg
    → rospy.Timer (20 Hz, 背景執行緒)
    → /mavros/setpoint_position/local
    → pfa_pos_control  (PFA_DES_PITCH=0, PFA_DES_ROLL=0)
    → pfa_att_control
    → 馬達

■ 方形頂點（以起飛位置為中心，ENU）
  v[0](SE) → v[1](NE) → v[2](NW) → v[3](SW) → v[0]  (逆時針 CCW)

執行方式：
  python3 square_traj_mavros.py
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
SQUARE_SIDE_M     = 1.0    # 方形邊長 (m)
SEG_SPEED_MPS     = 0.3    # 飛行速度 (m/s)
VERTEX_DWELL_S    = 1.5    # 各頂點停留時間 (s)，讓 yaw 完成轉向
NUM_LAPS          = 2      # 完整方形圈數
YAW_TRACK_PATH    = True   # True: yaw 追蹤前進方向; False: 固定 yaw=0°

HOVER_PHASE_DUR   = 5.0    # 起飛後穩定等待時間 (s)
ALT_TOL           = 0.10   # 高度穩定容忍 (m)
HOVER_STABLE_TIME = 3.0    # 連續穩定秒數
CTRL_HZ           = 20     # Timer 發布頻率

# Derived
_SEG_DUR_S = SQUARE_SIDE_M / SEG_SPEED_MPS         # 每段飛行時間
_LAP_DUR_S = 4 * (_SEG_DUR_S + VERTEX_DWELL_S)     # 每圈總時間

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
rospy.init_node('square_traj', anonymous=False)
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
rospy.loginfo(f"  Plan: alt={TARGET_ALT_M}m  side={SQUARE_SIDE_M}m  "
              f"speed={SEG_SPEED_MPS}m/s  "
              f"~{_SEG_DUR_S:.1f}s/side × 4 + {VERTEX_DWELL_S}s dwell  "
              f"× {NUM_LAPS} laps")

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
# Step 5: 穩定懸停，記錄方形中心座標
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 5] Hover hold {HOVER_PHASE_DUR:.0f} s (記錄方形中心)...")
t_end = rospy.Time.now() + rospy.Duration(HOVER_PHASE_DUR)
while not rospy.is_shutdown() and rospy.Time.now() < t_end:
    rate.sleep()

cx, cy, _ = get_xyz()
rospy.loginfo(f"  Square centre: ({cx:.3f}, {cy:.3f}) m (ENU)")

# ─────────────────────────────────────────────────────────────
# 建立方形頂點（以 cx,cy 為中心，邊長 SQUARE_SIDE_M）
# CCW 順序：SE → NE → NW → SW → SE
# ─────────────────────────────────────────────────────────────
S = SQUARE_SIDE_M / 2.0
verts = [
    (cx + S, cy - S),   # v[0] SE (+x, -y)
    (cx + S, cy + S),   # v[1] NE (+x, +y)
    (cx - S, cy + S),   # v[2] NW (-x, +y)
    (cx - S, cy - S),   # v[3] SW (-x, -y)
]
labels = ['SE', 'NE', 'NW', 'SW']
for k, (vx, vy) in enumerate(verts):
    rospy.loginfo(f"  v[{k}] {labels[k]}: ({vx:.3f}, {vy:.3f}) m")

def seg_yaw(src_i, dst_i):
    dx = verts[dst_i][0] - verts[src_i][0]
    dy = verts[dst_i][1] - verts[src_i][1]
    return math.atan2(dy, dx)

# ─────────────────────────────────────────────────────────────
# Step 6: 移動至 v[0]（cubic ease-in-out，3 s）
# ─────────────────────────────────────────────────────────────
APPROACH_DUR = 3.0
v0x, v0y  = verts[0]
init_yaw  = seg_yaw(0, 1) if YAW_TRACK_PATH else 0.0

rospy.loginfo(f"\n[Step 6] Approach v[0] ({labels[0]}: {v0x:.2f},{v0y:.2f}) in {APPROACH_DUR:.0f} s...")
t_app = rospy.Time.now()
while not rospy.is_shutdown():
    elapsed = (rospy.Time.now() - t_app).to_sec()
    frac    = min(elapsed / APPROACH_DUR, 1.0)
    s       = frac * frac * (3.0 - 2.0 * frac)   # cubic ease-in-out
    set_sp(make_sp(cx + s * (v0x - cx),
                   cy + s * (v0y - cy),
                   TARGET_ALT_M, yaw_rad=init_yaw))
    if frac >= 1.0: break
    rate.sleep()
rospy.loginfo(f"  At v[0].")

# ─────────────────────────────────────────────────────────────
# Step 7: 方形軌跡
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 7] Square trajectory ({NUM_LAPS} lap(s))  "
              f"~{_LAP_DUR_S:.0f} s/lap")

for lap in range(NUM_LAPS):
    rospy.loginfo(f"\n  ══ Lap {lap+1}/{NUM_LAPS} ══")

    for seg in range(4):
        src_i = seg
        dst_i = (seg + 1) % 4
        sx, sy = verts[src_i]
        dx, dy = verts[dst_i]
        yaw      = seg_yaw(src_i, dst_i) if YAW_TRACK_PATH else 0.0
        next_yaw = seg_yaw(dst_i, (dst_i + 1) % 4) if YAW_TRACK_PATH else 0.0

        rospy.loginfo(f"\n  ── {labels[src_i]} → {labels[dst_i]}  "
                      f"yaw={math.degrees(yaw):.0f}°  ({_SEG_DUR_S:.1f} s) ──")

        # 飛行段（線性插值）
        t_seg = last_log = rospy.Time.now()
        while not rospy.is_shutdown():
            now     = rospy.Time.now()
            elapsed = (now - t_seg).to_sec()
            frac    = min(elapsed / _SEG_DUR_S, 1.0)
            set_sp(make_sp(sx + frac * (dx - sx),
                           sy + frac * (dy - sy),
                           TARGET_ALT_M, yaw_rad=yaw))

            if (now - last_log).to_sec() >= 1.0:
                xn, yn, _ = get_xyz()
                r, p, _ = get_rpy_deg()
                rospy.loginfo(f"    {elapsed:5.1f}s  frac={frac:.2f}  "
                              f"cmd=({sx+frac*(dx-sx):.2f},{sy+frac*(dy-sy):.2f})  "
                              f"pos=({xn:.2f},{yn:.2f})  alt={get_altitude():.2f}m  "
                              f"roll={r:+.1f}°  pitch={p:+.1f}°")
                last_log = now
            if frac >= 1.0: break
            rate.sleep()

        # 頂點停留（yaw 轉向下一段）
        rospy.loginfo(f"  ✓ {labels[dst_i]} reached.  "
                      f"Dwell {VERTEX_DWELL_S:.1f} s  yaw→{math.degrees(next_yaw):.0f}°")
        t_dwell = rospy.Time.now()
        while not rospy.is_shutdown() and \
              (rospy.Time.now() - t_dwell).to_sec() < VERTEX_DWELL_S:
            set_sp(make_sp(dx, dy, TARGET_ALT_M, yaw_rad=next_yaw))
            rate.sleep()

rospy.loginfo(f"\n  ✓ Square trajectory complete ({NUM_LAPS} lap(s)).")

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
_sp_timer.shutdown()
rospy.loginfo("\n[Step 9] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(15.0)
rospy.loginfo("Done.")
