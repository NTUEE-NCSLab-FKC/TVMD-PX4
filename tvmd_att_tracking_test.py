#!/usr/bin/env python3
"""
TVMD Attitude Tracking Hover Test via pfa_att_control (MAVROS version)

Tests roll, pitch, and yaw trajectory tracking while hovering at a fixed altitude.
Sends sinusoidal attitude setpoints to pfa_att_control via MAVROS and logs
the tracking performance for each axis.

Test sequence:
  1. Takeoff to FLIGHT_HEIGHT using position control
  2. Roll  trajectory test  – sinusoidal roll,  pitch/yaw held at 0
  3. Pitch trajectory test  – sinusoidal pitch, roll/yaw  held at 0
  4. Yaw   trajectory test  – sinusoidal yaw,   roll/pitch held at 0
  5. Combined trajectory test – all three axes simultaneously
  6. Land

Frame convention (MAVROS / ROS ENU):
  - Orientation quaternion in AttitudeTarget is expressed in ENU/FLU.
  - MAVROS converts to NED/FRD before forwarding to PX4.
  - pfa_att_control receives vehicle_attitude_setpoint in NED/FRD frame.

Usage:
  Terminal 1: make px4_sitl gz_tvmd
  Terminal 2: roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
  Terminal 3: python3 tvmd_att_tracking_test.py
"""

import rospy
import math
import sys
import csv
import os
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

# ============ Configuration ============
FLIGHT_HEIGHT   = 1.0   # Hover altitude (m)
HOVER_THRUST    = 0.5   # Normalized thrust to maintain hover [0, 1] (tune for your TVMD)
TAKEOFF_TIMEOUT = 30.0  # Max time to reach hover height (s)

# Trajectory parameters (amplitudes in degrees, frequency in Hz)
ROLL_AMP_DEG    = 10.0   # Roll sinusoid amplitude
PITCH_AMP_DEG   = 10.0   # Pitch sinusoid amplitude
YAW_AMP_DEG     = 20.0   # Yaw sinusoid amplitude
TRAJ_FREQ_HZ    = 0.2    # Trajectory frequency for roll/pitch
YAW_FREQ_HZ     = 0.1    # Trajectory frequency for yaw (slower)
SETTLE_TIME     = 3.0    # Hold level attitude before/after each test (s)
TEST_DURATION   = 20.0   # Duration of each attitude test phase (s)
LOG_FREQ_HZ     = 50.0   # Control / logging rate (Hz)

# Log file
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "att_tracking_log.csv")

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


# ============ Helpers ============

def get_rpy_from_quaternion(q):
    """Return (roll, pitch, yaw) in radians from geometry_msgs Quaternion (ENU/FLU)."""
    return euler_from_quaternion([q.x, q.y, q.z, q.w])


def make_attitude_target(roll_rad, pitch_rad, yaw_rad, thrust):
    """
    Build an AttitudeTarget message in ENU/FLU frame.

    type_mask = 7  →  bits 0-2 set: ignore body-rate x/y/z,
                       use orientation quaternion + thrust.
    """
    msg = AttitudeTarget()
    msg.header = Header()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.type_mask       = 7   # ignore body rates; use orientation + thrust

    q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]
    msg.thrust = float(thrust)
    return msg


def make_pose_stamped(x, y, z, yaw_rad=0.0):
    """Build a PoseStamped setpoint in ENU frame (used for takeoff)."""
    pose = PoseStamped()
    pose.header.stamp    = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def wait_for_hover(pos_pub, target_z, timeout):
    """
    Publish position setpoint and wait until vehicle reaches target altitude.
    Returns True if successful within timeout.
    """
    rate      = rospy.Rate(20)
    start     = rospy.Time.now()
    last_log  = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start).to_sec()
        if elapsed > timeout:
            return False
        pos_pub.publish(make_pose_stamped(0.0, 0.0, target_z))
        if pose_received:
            pz = current_pose.pose.position.z
            if (rospy.Time.now() - last_log).to_sec() > 1.0:
                rospy.loginfo(f"  Climbing: z={pz:.2f} / {target_z:.2f} m")
                last_log = rospy.Time.now()
            if abs(pz - target_z) < 0.15:
                return True
        rate.sleep()
    return False


# ============ Attitude Trajectory Tests ============

def sinusoid(amplitude_rad, freq_hz, t):
    """Simple sinusoidal trajectory value at time t."""
    return amplitude_rad * math.sin(2.0 * math.pi * freq_hz * t)


def run_attitude_test(att_pub, phase_name,
                      roll_fn, pitch_fn, yaw_fn,
                      duration, log_writer):
    """
    Publish attitude setpoints for `duration` seconds, logging desired vs actual.

    Parameters
    ----------
    att_pub    : rospy.Publisher for AttitudeTarget
    phase_name : string label printed in log
    roll_fn    : callable(t) -> roll_rad
    pitch_fn   : callable(t) -> pitch_rad
    yaw_fn     : callable(t) -> yaw_rad
    duration   : float, seconds
    log_writer : csv.writer (or None to skip logging)
    """
    rate     = rospy.Rate(LOG_FREQ_HZ)
    t_start  = rospy.Time.now()
    errors   = []  # (roll_err, pitch_err, yaw_err) samples

    rospy.loginfo(f"  [{phase_name}] running for {duration:.0f} s ...")

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - t_start).to_sec()
        if t >= duration:
            break

        roll_d  = roll_fn(t)
        pitch_d = pitch_fn(t)
        yaw_d   = yaw_fn(t)

        att_pub.publish(make_attitude_target(roll_d, pitch_d, yaw_d, HOVER_THRUST))

        # Actual attitude
        if pose_received:
            r_a, p_a, y_a = get_rpy_from_quaternion(current_pose.pose.orientation)
        else:
            r_a, p_a, y_a = 0.0, 0.0, 0.0

        r_err = math.degrees(roll_d  - r_a)
        p_err = math.degrees(pitch_d - p_a)
        y_err = math.degrees(yaw_d   - y_a)
        # Wrap yaw error to [-180, 180]
        y_err = (y_err + 180.0) % 360.0 - 180.0
        errors.append((r_err, p_err, y_err))

        if log_writer is not None:
            log_writer.writerow([
                f"{t:.4f}", phase_name,
                f"{math.degrees(roll_d):.3f}",
                f"{math.degrees(pitch_d):.3f}",
                f"{math.degrees(yaw_d):.3f}",
                f"{math.degrees(r_a):.3f}",
                f"{math.degrees(p_a):.3f}",
                f"{math.degrees(y_a):.3f}",
                f"{r_err:.3f}", f"{p_err:.3f}", f"{y_err:.3f}",
            ])

        rate.sleep()

    # Print RMS errors
    if errors:
        n = len(errors)
        rms_r = math.sqrt(sum(e[0]**2 for e in errors) / n)
        rms_p = math.sqrt(sum(e[1]**2 for e in errors) / n)
        rms_y = math.sqrt(sum(e[2]**2 for e in errors) / n)
        rospy.loginfo(
            f"  [{phase_name}] RMS error – "
            f"roll={rms_r:.2f}° pitch={rms_p:.2f}° yaw={rms_y:.2f}°"
        )


def settle(att_pub, duration):
    """Hold level attitude for `duration` seconds (used between test phases)."""
    rate    = rospy.Rate(LOG_FREQ_HZ)
    t_end   = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < t_end:
        att_pub.publish(make_attitude_target(0.0, 0.0, 0.0, HOVER_THRUST))
        rate.sleep()


# ============ Main ============

def main():
    rospy.init_node('tvmd_att_tracking_test', anonymous=True)

    # --- Subscribers ---
    rospy.Subscriber('/mavros/state',                State,       state_cb)
    rospy.Subscriber('/mavros/local_position/pose',  PoseStamped, pose_cb)

    # --- Publishers ---
    pos_pub = rospy.Publisher(
        '/mavros/setpoint_position/local', PoseStamped,    queue_size=10)
    att_pub = rospy.Publisher(
        '/mavros/setpoint_raw/attitude',   AttitudeTarget, queue_size=10)

    # --- Services ---
    rospy.loginfo("Waiting for MAVROS services...")
    rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
    rospy.wait_for_service('/mavros/set_mode',   timeout=30)
    arming_client   = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode',   SetMode)

    rate = rospy.Rate(20)

    # --- Wait for FCU ---
    rospy.loginfo("Waiting for FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("FCU connected!")

    # --- Wait for local position ---
    rospy.loginfo("Waiting for local position estimate...")
    deadline = rospy.Time.now() + rospy.Duration(10)
    while not rospy.is_shutdown() and not pose_received:
        if rospy.Time.now() > deadline:
            rospy.logerr("No local position data. Is EKF2 running?")
            sys.exit(1)
        rate.sleep()
    rospy.loginfo("Local position available.")

    # --- Pre-arm: send setpoints so FCU accepts OFFBOARD ---
    rospy.loginfo("Sending initial position setpoints (5 s)...")
    for _ in range(100):
        if rospy.is_shutdown():
            return
        pos_pub.publish(make_pose_stamped(0.0, 0.0, FLIGHT_HEIGHT))
        rate.sleep()

    # --- Switch to OFFBOARD ---
    rospy.loginfo("Setting OFFBOARD mode...")
    offb_req = SetModeRequest()
    offb_req.custom_mode = 'OFFBOARD'
    last_req = rospy.Time.now()
    while not rospy.is_shutdown():
        pos_pub.publish(make_pose_stamped(0.0, 0.0, FLIGHT_HEIGHT))
        if (current_state.mode != 'OFFBOARD' and
                (rospy.Time.now() - last_req).to_sec() > 2.0):
            try:
                if set_mode_client(offb_req).mode_sent:
                    rospy.loginfo("  OFFBOARD request sent.")
            except rospy.ServiceException as e:
                rospy.logerr(f"  SetMode failed: {e}")
            last_req = rospy.Time.now()
        if current_state.mode == 'OFFBOARD':
            rospy.loginfo("  OFFBOARD confirmed!")
            break
        rate.sleep()

    # --- Arm ---
    rospy.loginfo("Arming...")
    last_req = rospy.Time.now()
    while not rospy.is_shutdown():
        pos_pub.publish(make_pose_stamped(0.0, 0.0, FLIGHT_HEIGHT))
        if (not current_state.armed and
                (rospy.Time.now() - last_req).to_sec() > 2.0):
            try:
                arm_req = CommandBoolRequest()
                arm_req.value = True
                if arming_client(arm_req).success:
                    rospy.loginfo("  Arm command accepted.")
            except rospy.ServiceException as e:
                rospy.logerr(f"  Arming failed: {e}")
            last_req = rospy.Time.now()
        if current_state.armed:
            rospy.loginfo("  Armed!")
            break
        rate.sleep()

    # --- Takeoff to hover height ---
    rospy.loginfo(f"Taking off to {FLIGHT_HEIGHT:.1f} m ...")
    if not wait_for_hover(pos_pub, FLIGHT_HEIGHT, TAKEOFF_TIMEOUT):
        rospy.logwarn(f"Did not reach {FLIGHT_HEIGHT:.1f} m within timeout – continuing anyway.")
    else:
        rospy.loginfo("Hover altitude reached!")

    # Hold level attitude (keep publishing so OFFBOARD stream never breaks)
    settle(att_pub, 2.0)

    # ============ Attitude Tracking Tests ============
    rospy.loginfo("")
    rospy.loginfo("=" * 60)
    rospy.loginfo("ATTITUDE TRACKING TEST  (via pfa_att_control)")
    rospy.loginfo(f"  Roll  : {ROLL_AMP_DEG:.1f}° @ {TRAJ_FREQ_HZ:.2f} Hz")
    rospy.loginfo(f"  Pitch : {PITCH_AMP_DEG:.1f}° @ {TRAJ_FREQ_HZ:.2f} Hz")
    rospy.loginfo(f"  Yaw   : {YAW_AMP_DEG:.1f}° @ {YAW_FREQ_HZ:.2f} Hz")
    rospy.loginfo(f"  Test duration per phase: {TEST_DURATION:.0f} s")
    rospy.loginfo(f"  Log file: {LOG_FILE}")
    rospy.loginfo("=" * 60)

    roll_amp  = math.radians(ROLL_AMP_DEG)
    pitch_amp = math.radians(PITCH_AMP_DEG)
    yaw_amp   = math.radians(YAW_AMP_DEG)

    # Pre-compute trajectory functions
    zero_fn   = lambda t: 0.0
    roll_fn   = lambda t: sinusoid(roll_amp,  TRAJ_FREQ_HZ, t)
    pitch_fn  = lambda t: sinusoid(pitch_amp, TRAJ_FREQ_HZ, t)
    yaw_fn    = lambda t: sinusoid(yaw_amp,   YAW_FREQ_HZ,  t)

    # Open CSV log
    log_file = open(LOG_FILE, 'w', newline='')
    writer   = csv.writer(log_file)
    writer.writerow([
        "t_s", "phase",
        "roll_d_deg", "pitch_d_deg", "yaw_d_deg",
        "roll_a_deg", "pitch_a_deg", "yaw_a_deg",
        "roll_err_deg", "pitch_err_deg", "yaw_err_deg",
    ])

    # --- Phase 1: Roll test ---
    rospy.loginfo("\n[Phase 1] Roll trajectory tracking (pitch=0, yaw=0)")
    settle(att_pub, SETTLE_TIME)
    run_attitude_test(att_pub, "roll_test",
                      roll_fn, zero_fn, zero_fn,
                      TEST_DURATION, writer)

    # --- Phase 2: Pitch test ---
    rospy.loginfo("\n[Phase 2] Pitch trajectory tracking (roll=0, yaw=0)")
    settle(att_pub, SETTLE_TIME)
    run_attitude_test(att_pub, "pitch_test",
                      zero_fn, pitch_fn, zero_fn,
                      TEST_DURATION, writer)

    # --- Phase 3: Yaw test ---
    rospy.loginfo("\n[Phase 3] Yaw trajectory tracking (roll=0, pitch=0)")
    settle(att_pub, SETTLE_TIME)
    run_attitude_test(att_pub, "yaw_test",
                      zero_fn, zero_fn, yaw_fn,
                      TEST_DURATION, writer)

    # --- Phase 4: Combined test ---
    rospy.loginfo("\n[Phase 4] Combined roll + pitch + yaw tracking")
    settle(att_pub, SETTLE_TIME)
    run_attitude_test(att_pub, "combined_test",
                      roll_fn, pitch_fn, yaw_fn,
                      TEST_DURATION, writer)

    log_file.close()
    rospy.loginfo(f"\nAll phases complete. Log saved to: {LOG_FILE}")

    # --- Land ---
    rospy.loginfo("Landing...")
    settle(att_pub, SETTLE_TIME)   # brief level hold before switching mode
    land_req = SetModeRequest()
    land_req.custom_mode = 'AUTO.LAND'
    try:
        set_mode_client(land_req)
        rospy.loginfo("AUTO.LAND mode set.")
    except rospy.ServiceException as e:
        rospy.logerr(f"  Land mode failed: {e}")

    rospy.sleep(15)
    rospy.loginfo("Done!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
