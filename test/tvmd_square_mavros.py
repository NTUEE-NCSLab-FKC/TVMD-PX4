#!/usr/bin/env python3
"""
TVMD Square Trajectory Test with Yaw at Corners (MAVROS version)
Takeoff -> Fly 1m x 1m square at 1m altitude with yaw rotation -> Land
At each corner, the vehicle rotates to face the direction of travel:
  Corner 0: yaw=0°   (facing +X)
  Corner 1: yaw=90°  (facing +Y)
  Corner 2: yaw=180° (facing -X)
  Corner 3: yaw=-90° (facing -Y)

Usage:
  rosrun tvmd_test tvmd_square_mavros.py
  # or simply:
  python3 tvmd_square_mavros.py
"""
import rospy
import math
import sys
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ============ Configuration ============
SIDE_LENGTH = 1.0       # Square side length (meters)
FLIGHT_HEIGHT = 1.0     # Flight height (meters)
VELOCITY = 0.5          # Flight velocity (m/s)
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


def get_yaw_from_pose(pose_msg):
    """Extract yaw from PoseStamped quaternion"""
    q = pose_msg.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def make_position_yaw_target(x, y, z, yaw):
    """Create PositionTarget message in NED-like frame.
    MAVROS converts ENU<->NED internally, so we publish in ENU:
      ENU x = NED y (East)
      ENU y = NED x (North)
      ENU z = -NED z (Up)
    But mavros/setpoint_raw/local with FRAME_LOCAL_NED handles
    the conversion automatically.
    """
    msg = PositionTarget()
    msg.header.stamp = rospy.Time.now()
    msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    # type_mask: use position + yaw, ignore velocity, acceleration, force, yaw_rate
    msg.type_mask = (
        PositionTarget.IGNORE_VX |
        PositionTarget.IGNORE_VY |
        PositionTarget.IGNORE_VZ |
        PositionTarget.IGNORE_AFX |
        PositionTarget.IGNORE_AFY |
        PositionTarget.IGNORE_AFZ |
        PositionTarget.IGNORE_YAW_RATE
    )
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    msg.yaw = yaw
    return msg


def fly_to_position(pub, x, y, z, yaw, timeout=20.0):
    """Fly to position with specified yaw, returns True if reached"""
    rate = rospy.Rate(20)
    start = rospy.Time.now()
    last_print = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start).to_sec()
        if elapsed > timeout:
            return False

        msg = make_position_yaw_target(x, y, z, yaw)
        pub.publish(msg)

        if pose_received:
            px = current_pose.pose.position.x
            py = current_pose.pose.position.y
            pz = current_pose.pose.position.z
            # MAVROS local_position/pose is in ENU
            # Our setpoint is in NED via setpoint_raw/local
            # For distance check, use NED coordinates from the pose
            # Actually, local_position/pose is ENU, so we need to convert
            # px_ned = py_enu, py_ned = px_enu, pz_ned = -pz_enu
            px_ned = py  # ENU.y -> NED.x (North)
            py_ned = px  # ENU.x -> NED.y (East)
            pz_ned = -pz  # ENU.z -> NED.z (Down)

            dist = math.sqrt((px_ned - x)**2 + (py_ned - y)**2 + (pz_ned - z)**2)

            now = rospy.Time.now()
            if (now - last_print).to_sec() > 1.0:
                cur_yaw = get_yaw_from_pose(current_pose)
                yaw_deg = math.degrees(cur_yaw)
                rospy.loginfo(
                    f"    pos=({px_ned:.2f}, {py_ned:.2f}, {pz_ned:.2f}) "
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
        msg = make_position_yaw_target(x, y, z, yaw)
        pub.publish(msg)
        rate.sleep()


def main():
    rospy.init_node('tvmd_square_mavros', anonymous=True)

    # Subscribers
    rospy.Subscriber('/mavros/state', State, state_cb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)

    # Publisher
    setpoint_pub = rospy.Publisher(
        '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

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
    rospy.loginfo(f"Connected to FCU!")

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

    H = -FLIGHT_HEIGHT  # NED: negative z = up

    # ---- Send setpoints before OFFBOARD ----
    rospy.loginfo("Sending initial setpoints (5 seconds)...")
    for i in range(100):
        if rospy.is_shutdown():
            return
        msg = make_position_yaw_target(0.0, 0.0, H, 0)
        setpoint_pub.publish(msg)
        rate.sleep()

    # ---- Switch to OFFBOARD ----
    rospy.loginfo("Setting OFFBOARD mode...")
    offb_req = SetModeRequest()
    offb_req.custom_mode = 'OFFBOARD'
    try:
        resp = set_mode_client(offb_req)
        if resp.mode_sent:
            rospy.loginfo("  OFFBOARD mode request sent!")
        else:
            rospy.logwarn("  OFFBOARD mode request failed, retrying...")
            # Keep sending setpoints and retry
            for _ in range(50):
                msg = make_position_yaw_target(0.0, 0.0, H, 0)
                setpoint_pub.publish(msg)
                rate.sleep()
            set_mode_client(offb_req)
    except rospy.ServiceException as e:
        rospy.logerr(f"  Set mode service call failed: {e}")

    # Wait for OFFBOARD confirmation
    timeout = rospy.Time.now() + rospy.Duration(5)
    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        msg = make_position_yaw_target(0.0, 0.0, H, 0)
        setpoint_pub.publish(msg)
        if current_state.mode == 'OFFBOARD':
            rospy.loginfo("  OFFBOARD mode confirmed!")
            break
        rate.sleep()

    # ---- Arm ----
    rospy.loginfo("Arming...")
    for i in range(20):
        msg = make_position_yaw_target(0.0, 0.0, H, 0)
        setpoint_pub.publish(msg)
        rate.sleep()

    arm_req = CommandBoolRequest()
    arm_req.value = True
    try:
        resp = arming_client(arm_req)
        if resp.success:
            rospy.loginfo("  Arm command accepted!")
        else:
            rospy.logwarn("  Arm command rejected, retrying...")
    except rospy.ServiceException as e:
        rospy.logerr(f"  Arming service call failed: {e}")

    # Wait for armed confirmation
    armed = False
    timeout = rospy.Time.now() + rospy.Duration(5)
    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        msg = make_position_yaw_target(0.0, 0.0, H, 0)
        setpoint_pub.publish(msg)
        if current_state.armed:
            armed = True
            break
        rate.sleep()

    if armed:
        rospy.loginfo("  Armed successfully!")
    else:
        rospy.logerr("  Failed to arm.")
        sys.exit(1)

    # ============ Square Trajectory with Yaw ============
    rospy.loginfo("")
    rospy.loginfo("=" * 50)
    rospy.loginfo("SQUARE TRAJECTORY WITH YAW ROTATION")
    rospy.loginfo(f"  Side: {SIDE_LENGTH}m, Height: {FLIGHT_HEIGHT}m")
    rospy.loginfo("=" * 50)

    L = SIDE_LENGTH

    waypoints = [
        (0, 0, H, 0,                      "Takeoff (yaw=0°)"),
        (L, 0, H, 0,                      "Corner 1 (yaw=0°, facing +X)"),
        (L, 0, H, math.pi/2,              "Rotate to 90°"),
        (L, L, H, math.pi/2,              "Corner 2 (yaw=90°, facing +Y)"),
        (L, L, H, math.pi,                "Rotate to 180°"),
        (0, L, H, math.pi,                "Corner 3 (yaw=180°, facing -X)"),
        (0, L, H, -math.pi/2,             "Rotate to -90°"),
        (0, 0, H, -math.pi/2,             "Corner 4 (yaw=-90°, facing -Y)"),
        (0, 0, H, 0,                      "Rotate back to 0°"),
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
                rospy.logwarn(
                    f"    Timeout. pos=({py:.2f}, {px:.2f}, {-pz:.2f}) [NED]")

    # ---- Land ----
    rospy.loginfo("Landing...")
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
