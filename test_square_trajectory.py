#!/usr/bin/env python3
"""
TVMD Square Trajectory Test (pymavlink, no ROS needed)
Takeoff -> Fly 1m x 1m square at 1m altitude -> Land
"""
import time
from pymavlink import mavutil

# Connect to PX4 SITL
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to PX4")


def set_position_target(x, y, z):
    """Send position setpoint in NED frame (z negative = up)"""
    master.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,         # type_mask (position only)
        x, y, z,                    # x, y, z (NED)
        0, 0, 0,                    # vx, vy, vz
        0, 0, 0,                    # afx, afy, afz
        0, 0                        # yaw, yaw_rate
    )


def get_local_position():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg:
        return msg.x, msg.y, msg.z
    return None, None, None


def reached(tx, ty, tz, tol=0.3):
    x, y, z = get_local_position()
    if x is None:
        return False
    return ((x - tx)**2 + (y - ty)**2 + (z - tz)**2) ** 0.5 < tol


def set_mode(mode):
    """Set PX4 flight mode"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode, 0, 0, 0, 0, 0)


def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)


# PX4 custom mode IDs
PX4_OFFBOARD = 6
PX4_AUTO_LAND = 0x06000004  # AUTO.LAND

# Square waypoints in NED (z negative = up)
waypoints = [
    (0.0, 0.0, -1.0),   # Takeoff
    (1.0, 0.0, -1.0),   # Forward 1m
    (1.0, 1.0, -1.0),   # Right 1m
    (0.0, 1.0, -1.0),   # Back 1m
    (0.0, 0.0, -1.0),   # Return home
]

# Send setpoints before switching to OFFBOARD (required by PX4)
print("Sending initial setpoints...")
for i in range(100):
    set_position_target(0, 0, -1.0)
    time.sleep(0.05)

# Switch to OFFBOARD and arm
print("Setting OFFBOARD mode...")
set_mode(PX4_OFFBOARD)
time.sleep(1)

print("Arming...")
arm()
time.sleep(3)

# Fly the square
for i, (x, y, z) in enumerate(waypoints):
    print(f"Flying to waypoint {i}: ({x}, {y}, {z})")
    timeout = time.time() + 15
    while time.time() < timeout:
        set_position_target(x, y, z)
        if reached(x, y, z):
            print(f"  Reached waypoint {i}, holding 3s...")
            hold_end = time.time() + 3
            while time.time() < hold_end:
                set_position_target(x, y, z)
                time.sleep(0.05)
            break
        time.sleep(0.05)
    else:
        print(f"  Timeout at waypoint {i}")

# Land
print("Landing...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
    0, 0, 0, 0, 0, 0, 0)
time.sleep(10)
print("Done!")
