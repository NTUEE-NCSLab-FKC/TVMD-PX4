#!/usr/bin/env python3
"""
TVMD Velocity Control Test (pymavlink)
Test velocity commands: hover, forward, yaw rotation
"""
import time
import sys
from pymavlink import mavutil

# Connect to PX4 SITL
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_component = 1
print(f"Connected! (system {master.target_system}, component {master.target_component})")


def set_velocity_target(vx, vy, vz, yaw_rate=0):
    """
    Send velocity setpoint in NED frame
    vx: forward velocity (m/s, positive = North)
    vy: right velocity (m/s, positive = East)
    vz: down velocity (m/s, positive = Down, negative = Up)
    yaw_rate: yaw angular rate (rad/s)
    """
    # type_mask: ignore position (bits 0-2), use velocity (bits 3-5 = 0)
    # ignore accel (bits 6-8), ignore yaw (bit 10), use yaw_rate (bit 11 = 0)
    type_mask = 0b0000011111000111  # velocity + yaw_rate only

    master.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,                    # x, y, z (ignored)
        vx, vy, vz,                 # vx, vy, vz (NED)
        0, 0, 0,                    # afx, afy, afz (ignored)
        0, yaw_rate                 # yaw (ignored), yaw_rate
    )


def set_position_velocity_target(x, y, z, vx, vy, vz):
    """Send both position and velocity setpoint"""
    # type_mask: use position (bits 0-2 = 0), use velocity (bits 3-5 = 0)
    type_mask = 0b0000111111000000  # position + velocity

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )


def get_local_position():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg:
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
    return None, None, None, None, None, None


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


# ---- Check local position ----
print("\nChecking local position estimate...")
x, y, z, vx, vy, vz = get_local_position()
if x is None:
    print("ERROR: No local position data.")
    sys.exit(1)
print(f"  Position: ({x:.2f}, {y:.2f}, {z:.2f})")
print(f"  Velocity: ({vx:.2f}, {vy:.2f}, {vz:.2f})")

# ---- Send setpoints before OFFBOARD ----
print("\nSending initial velocity setpoints (5 seconds)...")
for i in range(100):
    set_velocity_target(0, 0, -0.5)  # Slowly ascend
    time.sleep(0.05)

# ---- Switch to OFFBOARD ----
print("Setting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD mode confirmed!")
else:
    print("  WARNING: Could not confirm OFFBOARD mode, retrying...")
    for _ in range(50):
        set_velocity_target(0, 0, -0.5)
        time.sleep(0.05)
    set_mode_offboard()

# ---- Arm ----
print("Arming...")
for i in range(20):
    set_velocity_target(0, 0, -0.5)
    time.sleep(0.05)
arm()

armed = False
for i in range(100):
    set_velocity_target(0, 0, -0.5)
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    print("  Normal arm failed, trying force arm...")
    arm(force=True)
    for i in range(100):
        set_velocity_target(0, 0, -0.5)
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

# ---- Test velocity commands ----
print("\n" + "="*50)
print("VELOCITY CONTROL TEST")
print("="*50)

# Phase 1: Ascend to ~1m (climb at 0.3 m/s for 3 seconds)
print("\n[1] Ascending (vz = -0.3 m/s for 5 seconds)...")
start = time.time()
while time.time() - start < 5:
    set_velocity_target(0, 0, -0.3)
    x, y, z, vx, vy, vz = get_local_position()
    if x is not None and int(time.time() - start) % 1 == 0:
        print(f"    pos=({x:.2f}, {y:.2f}, {z:.2f}) vel=({vx:.2f}, {vy:.2f}, {vz:.2f})")
    time.sleep(0.05)

# Phase 2: Hover (zero velocity)
print("\n[2] Hovering (vx=vy=vz=0 for 3 seconds)...")
start = time.time()
while time.time() - start < 3:
    set_velocity_target(0, 0, 0)
    time.sleep(0.05)
x, y, z, vx, vy, vz = get_local_position()
print(f"    pos=({x:.2f}, {y:.2f}, {z:.2f})")

# Phase 3: Forward flight (vx = 0.5 m/s for 2 seconds = 1m forward)
print("\n[3] Forward (vx = 0.5 m/s for 2 seconds)...")
start = time.time()
while time.time() - start < 2:
    set_velocity_target(0.5, 0, 0)
    time.sleep(0.05)
x, y, z, vx, vy, vz = get_local_position()
print(f"    pos=({x:.2f}, {y:.2f}, {z:.2f})")

# Phase 4: Hover
print("\n[4] Hovering...")
start = time.time()
while time.time() - start < 2:
    set_velocity_target(0, 0, 0)
    time.sleep(0.05)

# Phase 5: Yaw rotation (rotate 90 degrees at 0.5 rad/s = ~1.8 seconds for 90deg)
print("\n[5] Yaw rotation (yaw_rate = 0.5 rad/s for 3 seconds)...")
start = time.time()
while time.time() - start < 3:
    set_velocity_target(0, 0, 0, yaw_rate=0.5)
    time.sleep(0.05)

# Phase 6: Hover
print("\n[6] Final hover...")
start = time.time()
while time.time() - start < 2:
    set_velocity_target(0, 0, 0)
    time.sleep(0.05)

# ---- Land ----
print("\nLanding...")
set_mode_land()
time.sleep(10)
print("Done!")
