#!/usr/bin/env python3
"""
TVMD Manual Control Test via MAVLink
Simulates RC joystick input for forward/backward/yaw control
"""
import time
import sys
from pymavlink import mavutil

# Connect
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_component = 1
print(f"Connected! (system {master.target_system})")


def send_rc(x=0, y=0, z=500, r=0):
    """
    Send manual control
    x: forward/backward (-1000 to 1000), positive = forward
    y: left/right (-1000 to 1000), positive = right
    z: throttle (0 to 1000), 500 = hover
    r: yaw (-1000 to 1000), positive = clockwise
    """
    master.mav.manual_control_send(
        master.target_system,
        int(x), int(y), int(z), int(r),
        0  # buttons
    )


def get_position():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if msg:
        return msg.x, msg.y, msg.z
    return None, None, None


def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # force arm


def set_mode_posctl():
    """Set Position Control mode (main=3 POSCTL)"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        3.0, 0.0, 0.0, 0.0, 0.0, 0.0)


# ---- Main Test ----
print("\n=== TVMD Manual Control Test ===")
print("Controls:")
print("  x: forward(+)/backward(-)")
print("  y: right(+)/left(-)")
print("  z: throttle (500=hover)")
print("  r: yaw right(+)/left(-)")

# Set mode and arm
print("\nSetting POSCTL mode...")
set_mode_posctl()
time.sleep(1)

print("Arming...")
arm()
time.sleep(2)

# Test sequence
tests = [
    ("Hover (3s)", 0, 0, 600, 0, 3),
    ("Forward (2s)", 400, 0, 600, 0, 2),
    ("Hover (2s)", 0, 0, 600, 0, 2),
    ("Right (2s)", 0, 400, 600, 0, 2),
    ("Hover (2s)", 0, 0, 600, 0, 2),
    ("Yaw Right (2s)", 0, 0, 600, 400, 2),
    ("Hover (2s)", 0, 0, 600, 0, 2),
    ("Descend (3s)", 0, 0, 300, 0, 3),
]

for name, x, y, z, r, duration in tests:
    print(f"\n[{name}] x={x}, y={y}, z={z}, r={r}")
    start = time.time()
    while time.time() - start < duration:
        send_rc(x, y, z, r)
        px, py, pz = get_position()
        if px is not None and int((time.time() - start) * 2) % 2 == 0:
            print(f"  pos: ({px:.2f}, {py:.2f}, {pz:.2f})")
        time.sleep(0.05)

print("\n=== Test Complete ===")
