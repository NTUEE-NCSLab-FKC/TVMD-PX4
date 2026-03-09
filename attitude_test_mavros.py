#!/usr/bin/env python3
"""
MAVROS Attitude Test in SITL
Takeoff -> Hover -> Roll/Pitch/Yaw attitude tests -> Land

Architecture
============
                setpoint_position/local          trajectory_setpoint
  MAVROS  ──────────────────────────────────>  pfa_pos_control  ──> vehicle_attitude_setpoint
          ──── setpoint_raw/attitude ─────>  (roll/pitch/yaw)         (thrust_body computed
                  (roll/pitch/yaw only)       read here by            from position error)
                                              pfa_pos_control          │
                                                                       ▼
                                                               pfa_att_control
                                                              (torque + thrust allocation)

MAVROS publishes two streams simultaneously:
  1. setpoint_position/local  – keeps PX4 in position-control OFFBOARD mode so that
                                pfa_pos_control runs and computes the gravity-compensating
                                3-D thrust_body in the body NED frame.
  2. setpoint_raw/attitude    – writes vehicle_attitude_setpoint.{roll,pitch,yaw}_body
                                which pfa_pos_control reads (ext_att_sp) to use as
                                attitude_des instead of the default (0, 0, yaw).

No manual thrust value is chosen in this script; pfa_pos_control owns all thrust.

Attitude test sequence (5 s each):
  Roll  +45° / -45°
  Pitch +45° / -45°
  Yaw   +90° / -90°

Usage:
  Terminal 1: make px4_sitl gz_tvmd
  Terminal 2: roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
  Terminal 3: python3 attitude_test_mavros.py
"""

import rospy
import math
import sys

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ============ Configuration ============
FLIGHT_HEIGHT   = 1.0    # Hover height (m, ENU +z = up)
HOLD_TIME       = 5.0    # Seconds to hold each attitude
TAKEOFF_TIMEOUT = 30.0   # Max seconds to reach target altitude

# ============ Global State ============
current_state = State()
current_pose  = PoseStamped()
pose_received = False


def state_cb(msg):
    global current_state
    current_state = msg


def pose_cb(msg):
    global current_pose, pose_received
    current_pose = msg
    pose_received = True


# ─────────────────────────────────────────────
# Message helpers
# ─────────────────────────────────────────────

def make_pose_stamped(x, y, z, yaw=0.0):
    """Position setpoint in ENU map frame (setpoint_position/local)."""
    pose = PoseStamped()
    pose.header.stamp    = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def make_attitude_target(roll, pitch, yaw):
    """
    Attitude-only setpoint (setpoint_raw/attitude).

    type_mask = 0b00111111 = 63
      Bits 0-2: ignore body rate (wx, wy, wz)
      Bits 3-5: ignore attitude  – NOT set, so attitude IS used
      Bit  6  : ignore thrust    – set, so thrust is ignored

    This tells pfa_pos_control the desired roll/pitch/yaw while leaving
    thrust_body computation entirely to pfa_pos_control.
    """
    att = AttitudeTarget()
    att.header            = Header()
    att.header.stamp      = rospy.Time.now()
    att.header.frame_id   = "base_footprint"
    # Ignore body-rate AND thrust; use only orientation.
    att.type_mask         = AttitudeTarget.IGNORE_ROLL_RATE \
                          | AttitudeTarget.IGNORE_PITCH_RATE \
                          | AttitudeTarget.IGNORE_YAW_RATE \
                          | AttitudeTarget.IGNORE_THRUST
    q = quaternion_from_euler(roll, pitch, yaw)
    att.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    att.thrust = 0.0  # ignored by type_mask
    return att


# ─────────────────────────────────────────────
# Flight helpers
# ─────────────────────────────────────────────

def takeoff_and_hover(pos_pub, target_z, timeout=TAKEOFF_TIMEOUT):
    """Use position setpoints until target altitude is reached."""
    rate       = rospy.Rate(20)
    start      = rospy.Time.now()
    last_print = rospy.Time.now()

    while not rospy.is_shutdown():
        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("Takeoff timeout!")
            return False

        pos_pub.publish(make_pose_stamped(0.0, 0.0, target_z))

        if pose_received:
            pz = current_pose.pose.position.z
            if (rospy.Time.now() - last_print).to_sec() >= 1.0:
                rospy.loginfo(f"  Altitude: {pz:.2f} m  /  {target_z:.2f} m")
                last_print = rospy.Time.now()
            if abs(pz - target_z) < 0.15:
                rospy.loginfo(f"  Target altitude reached ({pz:.2f} m)")
                return True

        rate.sleep()
    return False


def hold_position(pos_pub, x, y, z, yaw, duration):
    """Hold position setpoint for <duration> seconds."""
    rate     = rospy.Rate(20)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        pos_pub.publish(make_pose_stamped(x, y, z, yaw))
        rate.sleep()


def hold_attitude(pos_pub, att_pub, roll, pitch, yaw, duration, label):
    """
    Stream attitude command for <duration> seconds.

    Both topics are published each cycle:
      - setpoint_position/local  keeps pfa_pos_control in the thrust loop
      - setpoint_raw/attitude    carries the desired roll/pitch/yaw so
                                 pfa_pos_control uses them as attitude_des
    """
    rate       = rospy.Rate(20)
    end_time   = rospy.Time.now() + rospy.Duration(duration)
    last_print = rospy.Time.now()

    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        # Position setpoint at origin keeps pfa_pos_control active
        pos_pub.publish(make_pose_stamped(0.0, 0.0, FLIGHT_HEIGHT, yaw))
        # Attitude setpoint carries desired roll/pitch/yaw
        att_pub.publish(make_attitude_target(roll, pitch, yaw))

        remaining = (end_time - rospy.Time.now()).to_sec()
        if pose_received and (rospy.Time.now() - last_print).to_sec() >= 1.0:
            q = current_pose.pose.orientation
            r, p, y_cur = euler_from_quaternion([q.x, q.y, q.z, q.w])
            rospy.loginfo(
                f"  [{label}] "
                f"roll={math.degrees(r):+6.1f}°  "
                f"pitch={math.degrees(p):+6.1f}°  "
                f"yaw={math.degrees(y_cur):+7.1f}°  "
                f"({remaining:.1f}s left)"
            )
            last_print = rospy.Time.now()

        rate.sleep()


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

def main():
    rospy.init_node('attitude_test_mavros', anonymous=True)

    rospy.Subscriber('/mavros/state',               State,       state_cb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)

    pos_pub = rospy.Publisher('/mavros/setpoint_position/local',
                              PoseStamped,    queue_size=10)
    att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude',
                              AttitudeTarget, queue_size=10)

    rospy.loginfo("Waiting for MAVROS services...")
    rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
    rospy.wait_for_service('/mavros/set_mode',   timeout=30)
    arming_client   = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode',   SetMode)

    rate = rospy.Rate(20)

    rospy.loginfo("Waiting for FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("FCU connected!")

    rospy.loginfo("Waiting for local position estimate...")
    timeout = rospy.Time.now() + rospy.Duration(10)
    while not rospy.is_shutdown() and not pose_received:
        if rospy.Time.now() > timeout:
            rospy.logerr("No local position. Is EKF2 running?")
            sys.exit(1)
        rate.sleep()

    # ── Pre-stream position setpoints before requesting OFFBOARD ────
    rospy.loginfo("Streaming initial setpoints (5 s)...")
    for _ in range(100):
        if rospy.is_shutdown():
            return
        pos_pub.publish(make_pose_stamped(0.0, 0.0, FLIGHT_HEIGHT))
        rate.sleep()

    # ── OFFBOARD ────────────────────────────────────────────────────
    rospy.loginfo("Requesting OFFBOARD mode...")
    offb_req             = SetModeRequest()
    offb_req.custom_mode = 'OFFBOARD'
    last_req = rospy.Time.now()

    while not rospy.is_shutdown() and current_state.mode != 'OFFBOARD':
        pos_pub.publish(make_pose_stamped(0.0, 0.0, FLIGHT_HEIGHT))
        if (rospy.Time.now() - last_req).to_sec() > 2.0:
            try:
                resp = set_mode_client(offb_req)
                if resp.mode_sent:
                    rospy.loginfo("  OFFBOARD request sent.")
                last_req = rospy.Time.now()
            except rospy.ServiceException as e:
                rospy.logerr(f"  set_mode failed: {e}")
        rate.sleep()
    rospy.loginfo("OFFBOARD mode active!")

    # ── Arm ──────────────────────────────────────────────────────────
    rospy.loginfo("Arming...")
    last_req = rospy.Time.now()

    while not rospy.is_shutdown() and not current_state.armed:
        pos_pub.publish(make_pose_stamped(0.0, 0.0, FLIGHT_HEIGHT))
        if (rospy.Time.now() - last_req).to_sec() > 2.0:
            try:
                arm_req       = CommandBoolRequest()
                arm_req.value = True
                resp = arming_client(arm_req)
                if resp.success:
                    rospy.loginfo("  Arm command accepted.")
                last_req = rospy.Time.now()
            except rospy.ServiceException as e:
                rospy.logerr(f"  arming failed: {e}")
        rate.sleep()
    rospy.loginfo("Armed!")

    # ── Takeoff ──────────────────────────────────────────────────────
    rospy.loginfo(f"\nTaking off to {FLIGHT_HEIGHT:.1f} m...")
    if not takeoff_and_hover(pos_pub, FLIGHT_HEIGHT):
        rospy.logerr("Takeoff failed.")
        sys.exit(1)

    rospy.loginfo("Hovering at origin. Stabilising 3 s...")
    hold_position(pos_pub, 0.0, 0.0, FLIGHT_HEIGHT, 0.0, 3.0)

    # ── Attitude Test Sequence ───────────────────────────────────────
    rospy.loginfo("")
    rospy.loginfo("=" * 55)
    rospy.loginfo("  ATTITUDE TEST SEQUENCE")
    rospy.loginfo("  Thrust allocation: pfa_pos_control + pfa_att_control")
    rospy.loginfo(f"  Hold time per step: {HOLD_TIME:.0f} s")
    rospy.loginfo("=" * 55)

    DEG45 = math.radians(45.0)
    DEG90 = math.radians(90.0)

    # (roll, pitch, yaw, label)
    # Thrust is NOT specified here — pfa_pos_control computes it from
    # position error + gravity compensation, rotated into body frame.
    test_sequence = [
        # ── Roll ────────────────────────────────────────────────────
        ( DEG45,   0.0,    0.0,  "Roll  +45°"),
        (-DEG45,   0.0,    0.0,  "Roll  -45°"),
        # ── Pitch ───────────────────────────────────────────────────
        (  0.0,  DEG45,    0.0,  "Pitch +45°"),
        (  0.0, -DEG45,    0.0,  "Pitch -45°"),
        # ── Yaw ─────────────────────────────────────────────────────
        (  0.0,    0.0,  DEG90,  "Yaw   +90°"),
        (  0.0,    0.0, -DEG90,  "Yaw   -90°"),
    ]

    for roll, pitch, yaw, label in test_sequence:
        rospy.loginfo(f"\n>>> {label}")
        hold_attitude(pos_pub, att_pub, roll, pitch, yaw, HOLD_TIME, label)

        # Level-hover recovery via position control (2 s)
        rospy.loginfo("  Recovery hover (2 s)...")
        hold_position(pos_pub, 0.0, 0.0, FLIGHT_HEIGHT, 0.0, 2.0)

    # ── Return to level hover before landing ─────────────────────────
    rospy.loginfo("\nAll attitude tests complete. Level hover 3 s...")
    hold_position(pos_pub, 0.0, 0.0, FLIGHT_HEIGHT, 0.0, 3.0)

    # ── Land ─────────────────────────────────────────────────────────
    rospy.loginfo("Switching to AUTO.LAND...")
    land_req             = SetModeRequest()
    land_req.custom_mode = 'AUTO.LAND'
    try:
        set_mode_client(land_req)
    except rospy.ServiceException as e:
        rospy.logerr(f"Land mode failed: {e}")

    rospy.sleep(15)
    rospy.loginfo("Attitude test sequence finished.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
