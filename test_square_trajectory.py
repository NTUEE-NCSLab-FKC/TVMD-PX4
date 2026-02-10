#!/usr/bin/env python3
"""
TVMD Square Trajectory Test (pymavlink, no ROS needed)
Takeoff -> Fly 1m x 1m square at 1m altitude -> Land
"""
import time
import sys
from pymavlink import mavutil

# Connect to PX4 SITL
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
# PX4 autopilot is component 1
master.target_system = master.target_system
master.target_component = 1
print(f"Connected! (system {master.target_system}, component {master.target_component})")


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
    dist = ((x - tx)**2 + (y - ty)**2 + (z - tz)**2) ** 0.5
    return dist < tol


def wait_for_arm(timeout=10):
    """Wait until vehicle is armed"""
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def wait_for_mode(target_main_mode, timeout=5):
    """Wait until vehicle enters the expected PX4 custom mode"""
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            # PX4 custom_mode: bits 16-23 = main mode
            main_mode = (msg.custom_mode >> 16) & 0xFF
            sub_mode = (msg.custom_mode >> 24) & 0xFF
            if main_mode == target_main_mode:
                return True
    return False


def set_mode_offboard():
    """Set OFFBOARD mode (PX4 custom main mode = 6)"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        6.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def set_mode_land():
    """Set AUTO.LAND mode (PX4 main=4 AUTO, sub=6 LAND)"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0)


def arm(force=False):
    """Arm the vehicle. force=True bypasses preflight checks."""
    p2 = 21196.0 if force else 0.0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, p2, 0.0, 0.0, 0.0, 0.0, 0.0)


# ---- Check local position is valid ----
print("\nChecking local position estimate...")
x, y, z = get_local_position()
if x is None:
    print("ERROR: No local position data. Is EKF2 running?")
    sys.exit(1)
print(f"  Current position: ({x:.2f}, {y:.2f}, {z:.2f})")

# ---- Send setpoints before OFFBOARD (PX4 requires this) ----
print("\nSending initial setpoints (5 seconds)...")
for i in range(100):
    set_position_target(0.0, 0.0, -1.0)
    time.sleep(0.05)

# ---- Switch to OFFBOARD ----
print("Setting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):  # PX4 OFFBOARD main mode = 6
    print("  OFFBOARD mode confirmed!")
else:
    print("  WARNING: Could not confirm OFFBOARD mode, retrying...")
    # Retry: send more setpoints and try again
    for _ in range(50):
        set_position_target(0.0, 0.0, -1.0)
        time.sleep(0.05)
    set_mode_offboard()
    if wait_for_mode(6):
        print("  OFFBOARD mode confirmed on retry!")
    else:
        print("  WARNING: Still not in OFFBOARD mode, continuing...")

# Keep sending setpoints while arming
print("Arming...")
for i in range(20):
    set_position_target(0.0, 0.0, -1.0)
    time.sleep(0.05)
arm()

# Keep sending setpoints and wait for arm
armed = False
for i in range(100):
    set_position_target(0.0, 0.0, -1.0)
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    print("  Normal arm failed, trying force arm...")
    arm(force=True)
    for i in range(100):
        set_position_target(0.0, 0.0, -1.0)
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.05)

if armed:
    print("  Armed successfully!")
else:
    print("  ERROR: Failed to arm. Check 'commander check' in pxh.")
    sys.exit(1)

# ---- Fly the square ----
# Square waypoints in NED (z negative = up)
waypoints = [
    (0.0, 0.0, -1.0),   # Takeoff / hover
    (1.0, 0.0, -1.0),   # Forward 1m
    (1.0, 1.0, -1.0),   # Right 1m
    (0.0, 1.0, -1.0),   # Back 1m
    (0.0, 0.0, -1.0),   # Return home
]

last_print = 0
for i, (wx, wy, wz) in enumerate(waypoints):
    label = ["Takeoff/Hover", "Forward", "Right", "Back", "Return"][i]
    print(f"\n[{i}] {label} -> ({wx}, {wy}, {wz})")
    timeout = time.time() + 20
    while time.time() < timeout:
        set_position_target(wx, wy, wz)
        x, y, z = get_local_position()
        if x is not None:
            dist = ((x - wx)**2 + (y - wy)**2 + (z - wz)**2) ** 0.5
            # Print position every 2 seconds
            now = time.time()
            if now - last_print > 2.0:
                print(f"  pos=({x:.2f}, {y:.2f}, {z:.2f}) dist={dist:.2f}")
                last_print = now
            if dist < 0.3:
                print(f"  Reached! pos=({x:.2f}, {y:.2f}, {z:.2f}), holding 3s...")
                hold_end = time.time() + 3
                while time.time() < hold_end:
                    set_position_target(wx, wy, wz)
                    time.sleep(0.05)
                break
        time.sleep(0.05)
    else:
        if x is not None:
            print(f"  Timeout. Last pos=({x:.2f}, {y:.2f}, {z:.2f})")
        else:
            print(f"  Timeout. No position data.")

# ---- Land ----
print("\nLanding...")
set_mode_land()
time.sleep(15)
print("Done!")
