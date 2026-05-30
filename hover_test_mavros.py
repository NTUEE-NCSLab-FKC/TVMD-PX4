#!/usr/bin/env python3
"""
Hover Test (MAVROS / ROS 1)
============================
最簡單的起飛懸停腳本：起飛至 0.5 m，維持 30 秒，AUTO.LAND 降落。
用於驗證 MAVROS → 實體機 連線與解鎖是否正常。

修正：rospy.Timer 在背景執行緒以 20 Hz 持續發布 setpoint，
確保 arm() / set_mode() 等阻塞服務呼叫期間 PX4 不會
因 offboard signal timeout (COM_OF_LOSS_T) 而拒絕解鎖。

執行方式：
  python3 hover_test_mavros.py
"""

import sys
import threading
import rospy
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M  = 0.5    # 懸停高度 (m)
HOVER_DUR_S   = 30.0   # 懸停時間 (s)
CTRL_HZ       = 20     # setpoint 發布頻率

# ─────────────────────────────────────────────────────────────
# State
# ─────────────────────────────────────────────────────────────
_state = State()
_pose  = PoseStamped()

def _cb_state(msg): global _state; _state = msg
def _cb_pose(msg):  global _pose;  _pose  = msg

def get_alt(): return _pose.pose.position.z

# ─────────────────────────────────────────────────────────────
# Shared setpoint (Timer 讀取；主執行緒寫入)
# ─────────────────────────────────────────────────────────────
_sp      = {'x': 0.0, 'y': 0.0, 'z': TARGET_ALT_M}
_sp_lock = threading.Lock()

def update_sp(x=None, y=None, z=None):
    with _sp_lock:
        if x is not None: _sp['x'] = x
        if y is not None: _sp['y'] = y
        if z is not None: _sp['z'] = z

def make_sp(x=0.0, y=0.0, z=TARGET_ALT_M, yaw=0.0):
    sp = PoseStamped()
    sp.header.stamp    = rospy.Time.now()
    sp.header.frame_id = 'map'
    sp.pose.position.x = x
    sp.pose.position.y = y
    sp.pose.position.z = z
    q = quaternion_from_euler(0.0, 0.0, yaw)
    sp.pose.orientation.x = q[0]
    sp.pose.orientation.y = q[1]
    sp.pose.orientation.z = q[2]
    sp.pose.orientation.w = q[3]
    return sp

def _timer_publish(_event):
    with _sp_lock:
        x, y, z = _sp['x'], _sp['y'], _sp['z']
    sp_pub.publish(make_sp(x, y, z))

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
rospy.init_node('hover_test', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# Timer 在背景執行緒以 20 Hz 持續發布 setpoint
# 確保 arm() / set_mode() 阻塞期間 PX4 不會 timeout
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_publish)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected!  mode={_state.mode}  armed={_state.armed}")

# ─────────────────────────────────────────────────────────────
# Step 0: PFA 姿態參數歸零
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
rospy.loginfo("  PFA_DES_ROLL  ... " + ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH ... " + ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

# ─────────────────────────────────────────────────────────────
# Step 1: 串流 setpoint（OFFBOARD 前置條件；Timer 已在背景發布）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 1] Streaming setpoints for 5 s (z={TARGET_ALT_M} m)...")
rospy.sleep(5.0)   # Timer 持續發布，主執行緒只需等待

# ─────────────────────────────────────────────────────────────
# Step 2: 切換 OFFBOARD
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 2] Requesting OFFBOARD mode...")
set_mode('OFFBOARD')   # Timer 在背景持續發布，不中斷

t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_state.mode}")

# ─────────────────────────────────────────────────────────────
# Step 3: 解鎖
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 3] Arming...")
arm()   # 阻塞服務呼叫；Timer 在背景持續發布 setpoint，PX4 不會 timeout

t0 = rospy.Time.now()
while not rospy.is_shutdown() and not _state.armed:
    if (rospy.Time.now() - t0).to_sec() > 10.0:
        rospy.logerr("  ✗ Failed to arm. Check QGC for pre-arm errors.")
        _sp_timer.shutdown()
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  ✓ Armed!")

# ─────────────────────────────────────────────────────────────
# Step 4: 懸停 30 秒
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Hovering at {TARGET_ALT_M} m for {HOVER_DUR_S:.0f} s...")
rospy.loginfo(f"  {'Time':>6}  {'Alt':>6}")
rospy.loginfo("  " + "─" * 16)

t0       = rospy.Time.now()
last_log = rospy.Time.now()

while not rospy.is_shutdown():
    now     = rospy.Time.now()
    elapsed = (now - t0).to_sec()

    if (now - last_log).to_sec() > 2.0:
        rospy.loginfo(f"  {elapsed:6.1f}s  {get_alt():6.2f}m")
        last_log = now

    if elapsed >= HOVER_DUR_S:
        break
    rate.sleep()

# ─────────────────────────────────────────────────────────────
# Step 5: 降落
# ─────────────────────────────────────────────────────────────
_sp_timer.shutdown()
rospy.loginfo("\n[Step 5] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
rospy.sleep(10.0)
rospy.loginfo("Done.")
