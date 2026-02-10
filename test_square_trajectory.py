#!/usr/bin/env python3
"""
TVMD Square Trajectory Test with Yaw at Corners
Takeoff -> Fly 1m x 1m square at 1m altitude with yaw rotation -> Land

At each corner, the vehicle rotates to face the direction of travel:
  Corner 0: yaw=0°   (facing +X)
  Corner 1: yaw=90°  (facing +Y)
  Corner 2: yaw=180° (facing -X)
  Corner 3: yaw=-90° (facing -Y)
"""
import time
import sys
import math
from pymavlink import mavutil

# ============ Configuration ============
SIDE_LENGTH = 1.0       # Square side length (meters)
FLIGHT_HEIGHT = 1.0     # Flight height (meters)
VELOCITY = 0.5          # Flight velocity (m/s)
CORNER_HOLD_TIME = 2.0  # Time to hold at each corner (seconds)

# Connect to PX4 SITL
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system = master.target_system
master.target_component = 1
print(f"Connected! (system {master.target_system}, component {master.target_component})")


def set_position_yaw_target(x, y, z, yaw=0):
    """Send position and yaw setpoint in NED frame (z negative = up)"""
    # type_mask bits: 0-2=pos, 3-5=vel, 6-8=acc, 9=force, 10=yaw, 11=yaw_rate
    # Set bit to 0 to USE, set to 1 to IGNORE
    # Position + yaw: ignore vel(3-5), acc(6-8), force(9), yaw_rate(11)
    type_mask = 0b0000101111111000  # position + yaw (bit 10=0)
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
        x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0)


def set_velocity_yaw_target(vx, vy, vz, yaw=0):
    """Send velocity and yaw setpoint in NED frame"""
    # Velocity + yaw: ignore pos(0-2), acc(6-8), force(9), yaw_rate(11)
    type_mask = 0b0000101111000111  # velocity + yaw (bit 10=0)
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
        0, 0, 0, vx, vy, vz, 0, 0, 0, yaw, 0)


def get_local_position():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg:
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
    return None, None, None, None, None, None


def get_attitude():
    """Get current attitude (roll, pitch, yaw)"""
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        return msg.roll, msg.pitch, msg.yaw
    return None, None, None


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def wait_for_mode(target_main_mode, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            main_mode = (msg.custom_mode >> 16) & 0xFF
            if main_mode == target_main_mode:
                return True
    return False


def set_mode_offboard():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        6.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def set_mode_land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0)


def arm(force=False):
    p2 = 21196.0 if force else 0.0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, p2, 0.0, 0.0, 0.0, 0.0, 0.0)


def fly_to_position(x, y, z, yaw, timeout=20):
    """Fly to position with specified yaw, returns True if reached"""
    start = time.time()
    last_print = 0
    while time.time() - start < timeout:
        set_position_yaw_target(x, y, z, yaw)
        px, py, pz, _, _, _ = get_local_position()
        if px is not None:
            dist = math.sqrt((px - x)**2 + (py - y)**2 + (pz - z)**2)
            now = time.time()
            if now - last_print > 1.0:
                _, _, cur_yaw = get_attitude()
                yaw_deg = math.degrees(cur_yaw) if cur_yaw else 0
                print(f"    pos=({px:.2f}, {py:.2f}, {pz:.2f}) yaw={yaw_deg:.0f}° dist={dist:.2f}")
                last_print = now
            if dist < 0.3:
                return True
        time.sleep(0.05)
    return False


def hold_position(x, y, z, yaw, duration):
    """Hold at position with yaw for specified duration"""
    end_time = time.time() + duration
    while time.time() < end_time:
        set_position_yaw_target(x, y, z, yaw)
        time.sleep(0.05)


# ---- Check local position ----
print("\nChecking local position estimate...")
x, y, z, _, _, _ = get_local_position()
if x is None:
    print("ERROR: No local position data. Is EKF2 running?")
    sys.exit(1)
print(f"  Current position: ({x:.2f}, {y:.2f}, {z:.2f})")

# ---- Send setpoints before OFFBOARD ----
print("\nSending initial setpoints (5 seconds)...")
for i in range(100):
    set_position_yaw_target(0.0, 0.0, -FLIGHT_HEIGHT, 0)
    time.sleep(0.05)

# ---- Switch to OFFBOARD ----
print("Setting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD mode confirmed!")
else:
    print("  Retrying...")
    for _ in range(50):
        set_position_yaw_target(0.0, 0.0, -FLIGHT_HEIGHT, 0)
        time.sleep(0.05)
    set_mode_offboard()

# ---- Arm ----
print("Arming...")
for i in range(20):
    set_position_yaw_target(0.0, 0.0, -FLIGHT_HEIGHT, 0)
    time.sleep(0.05)
arm()

armed = False
for i in range(100):
    set_position_yaw_target(0.0, 0.0, -FLIGHT_HEIGHT, 0)
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    print("  Trying force arm...")
    arm(force=True)
    for i in range(100):
        set_position_yaw_target(0.0, 0.0, -FLIGHT_HEIGHT, 0)
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.05)

if armed:
    print("  Armed successfully!")
else:
    print("  ERROR: Failed to arm.")
    sys.exit(1)

# ============ Square Trajectory with Yaw ============
print("\n" + "="*50)
print("SQUARE TRAJECTORY WITH YAW ROTATION")
print(f"  Side: {SIDE_LENGTH}m, Height: {FLIGHT_HEIGHT}m")
print("="*50)

# Waypoints: (x, y, z, yaw)
# z is negative in NED for altitude
# yaw is the heading to face the next waypoint direction
L = SIDE_LENGTH
H = -FLIGHT_HEIGHT

waypoints = [
    # (x, y, z, yaw_rad, description)
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
    print(f"\n[{i}] {desc}")
    print(f"    Target: ({wx:.1f}, {wy:.1f}, {wz:.1f}) yaw={math.degrees(wyaw):.0f}°")

    if fly_to_position(wx, wy, wz, wyaw):
        print(f"    Reached! Holding {CORNER_HOLD_TIME}s...")
        hold_position(wx, wy, wz, wyaw, CORNER_HOLD_TIME)
    else:
        px, py, pz, _, _, _ = get_local_position()
        if px:
            print(f"    Timeout. pos=({px:.2f}, {py:.2f}, {pz:.2f})")

# ---- Land ----
print("\nLanding...")
set_mode_land()
time.sleep(15)
print("Done!")
