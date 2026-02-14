#!/usr/bin/env python3
"""
TVMD Square Trajectory Test with Yaw at Corners (MAVROS version)
Takeoff -> Fly 1m x 1m square at 1m altitude with yaw rotation -> Land

Uses ENU coordinate frame (MAVROS default):
  X = East, Y = North, Z = Up

Usage:
  Terminal 1: make px4_sitl gz_tvmd
  Terminal 2: roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
  Terminal 3: python3 tvmd_square_mavros.py
"""
import rospy
import math
import sys
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ============ Configuration ============
SIDE_LENGTH = 1.0       # Square side length (meters)
FLIGHT_HEIGHT = 1.0     # Flight height (meters)
CORNER_HOLD_TIME = 2.0  # Time to hold at each corner (seconds)

# ============ Global State ============
current_state = State()
current_pose = PoseStamped()
pose_received = False


def state_cb(msg):
    global current_state
    current_state = msg


def pose_cb(msg):
    global current_pose, pose_received
    current_pose = msg
    pose_received = True


def get_yaw_from_quaternion(q):
    """Extract yaw from quaternion"""
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def make_pose_stamped(x, y, z, yaw):
    """Create PoseStamped message in ENU frame"""
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    # Convert yaw to quaternion
    q = quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    return pose


def fly_to_position(pub, x, y, z, yaw, timeout=20.0):
    """Fly to position (ENU) with specified yaw, returns True if reached"""
    rate = rospy.Rate(20)
    start = rospy.Time.now()
    last_print = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start).to_sec()
        if elapsed > timeout:
            return False

        # Publish setpoint
        msg = make_pose_stamped(x, y, z, yaw)
        pub.publish(msg)

        if pose_received:
            # Current position in ENU
            px = current_pose.pose.position.x
            py = current_pose.pose.position.y
            pz = current_pose.pose.position.z

            # Distance to target (all in ENU)
            dist = math.sqrt((px - x)**2 + (py - y)**2 + (pz - z)**2)

            now = rospy.Time.now()
            if (now - last_print).to_sec() > 1.0:
                cur_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
                yaw_deg = math.degrees(cur_yaw)
                rospy.loginfo(
                    f"    pos=({px:.2f}, {py:.2f}, {pz:.2f}) "
                    f"yaw={yaw_deg:.0f}° dist={dist:.2f}")
                last_print = now

            if dist < 0.3:
                return True

        rate.sleep()
    return False


def hold_position(pub, x, y, z, yaw, duration):
    """Hold at position with yaw for specified duration"""
    rate = rospy.Rate(20)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        msg = make_pose_stamped(x, y, z, yaw)
        pub.publish(msg)
        rate.sleep()


def main():
    rospy.init_node('tvmd_square_mavros', anonymous=True)

    # Subscribers
    rospy.Subscriber('/mavros/state', State, state_cb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)

    # Publisher - use setpoint_position/local (ENU frame)
    setpoint_pub = rospy.Publisher(
        '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Service proxies
    rospy.loginfo("Waiting for MAVROS services...")
    rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
    rospy.wait_for_service('/mavros/set_mode', timeout=30)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    rate = rospy.Rate(20)

    # Wait for FCU connection
    rospy.loginfo("Waiting for FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("Connected to FCU!")

    # ---- Check local position ----
    rospy.loginfo("Checking local position estimate...")
    timeout = rospy.Time.now() + rospy.Duration(5)
    while not rospy.is_shutdown() and not pose_received:
        if rospy.Time.now() > timeout:
            rospy.logerr("No local position data. Is EKF2 running?")
            sys.exit(1)
        rate.sleep()

    px = current_pose.pose.position.x
    py = current_pose.pose.position.y
    pz = current_pose.pose.position.z
    rospy.loginfo(f"  Current position (ENU): ({px:.2f}, {py:.2f}, {pz:.2f})")

    H = FLIGHT_HEIGHT  # ENU: positive z = up

    # ---- Send setpoints before OFFBOARD ----
    rospy.loginfo("Sending initial setpoints (5 seconds)...")
    for i in range(100):
        if rospy.is_shutdown():
            return
        msg = make_pose_stamped(0.0, 0.0, H, 0)
        setpoint_pub.publish(msg)
        rate.sleep()

    # ---- Switch to OFFBOARD ----
    rospy.loginfo("Setting OFFBOARD mode...")
    offb_req = SetModeRequest()
    offb_req.custom_mode = 'OFFBOARD'

    # Keep trying until OFFBOARD mode is set
    last_req = rospy.Time.now()
    while not rospy.is_shutdown():
        msg = make_pose_stamped(0.0, 0.0, H, 0)
        setpoint_pub.publish(msg)

        if current_state.mode != 'OFFBOARD' and (rospy.Time.now() - last_req).to_sec() > 2.0:
            try:
                resp = set_mode_client(offb_req)
                if resp.mode_sent:
                    rospy.loginfo("  OFFBOARD mode request sent!")
                last_req = rospy.Time.now()
            except rospy.ServiceException as e:
                rospy.logerr(f"  Set mode service call failed: {e}")

        if current_state.mode == 'OFFBOARD':
            rospy.loginfo("  OFFBOARD mode confirmed!")
            break

        rate.sleep()

    # ---- Arm ----
    rospy.loginfo("Arming...")
    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        msg = make_pose_stamped(0.0, 0.0, H, 0)
        setpoint_pub.publish(msg)

        if not current_state.armed and (rospy.Time.now() - last_req).to_sec() > 2.0:
            try:
                arm_req = CommandBoolRequest()
                arm_req.value = True
                resp = arming_client(arm_req)
                if resp.success:
                    rospy.loginfo("  Arm command accepted!")
                last_req = rospy.Time.now()
            except rospy.ServiceException as e:
                rospy.logerr(f"  Arming service call failed: {e}")

        if current_state.armed:
            rospy.loginfo("  Armed successfully!")
            break

        rate.sleep()

    # ============ Square Trajectory with Yaw ============
    rospy.loginfo("")
    rospy.loginfo("=" * 50)
    rospy.loginfo("SQUARE TRAJECTORY WITH YAW ROTATION (ENU)")
    rospy.loginfo(f"  Side: {SIDE_LENGTH}m, Height: {FLIGHT_HEIGHT}m")
    rospy.loginfo("=" * 50)

    L = SIDE_LENGTH

    # Waypoints in ENU: (x=East, y=North, z=Up, yaw)
    # yaw: 0=East(+X), pi/2=North(+Y), pi=West(-X), -pi/2=South(-Y)
    waypoints = [
        (0, 0, H, 0,                      "Takeoff (yaw=0°, facing East)"),
        (L, 0, H, 0,                      "Move East to Corner 1"),
        (L, 0, H, math.pi/2,              "Rotate to face North"),
        (L, L, H, math.pi/2,              "Move North to Corner 2"),
        (L, L, H, math.pi,                "Rotate to face West"),
        (0, L, H, math.pi,                "Move West to Corner 3"),
        (0, L, H, -math.pi/2,             "Rotate to face South"),
        (0, 0, H, -math.pi/2,             "Move South to Corner 4"),
        (0, 0, H, 0,                      "Rotate back to face East"),
    ]

    for i, (wx, wy, wz, wyaw, desc) in enumerate(waypoints):
        rospy.loginfo(f"\n[{i}] {desc}")
        rospy.loginfo(
            f"    Target: ({wx:.1f}, {wy:.1f}, {wz:.1f}) "
            f"yaw={math.degrees(wyaw):.0f}°")

        if fly_to_position(setpoint_pub, wx, wy, wz, wyaw):
            rospy.loginfo(f"    Reached! Holding {CORNER_HOLD_TIME}s...")
            hold_position(setpoint_pub, wx, wy, wz, wyaw, CORNER_HOLD_TIME)
        else:
            if pose_received:
                px = current_pose.pose.position.x
                py = current_pose.pose.position.y
                pz = current_pose.pose.position.z
                rospy.logwarn(f"    Timeout. pos=({px:.2f}, {py:.2f}, {pz:.2f})")

    # ---- Land ----
    rospy.loginfo("\nLanding...")
    land_req = SetModeRequest()
    land_req.custom_mode = 'AUTO.LAND'
    try:
        set_mode_client(land_req)
    except rospy.ServiceException as e:
        rospy.logerr(f"  Land mode service call failed: {e}")

    rospy.sleep(15)
    rospy.loginfo("Done!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
