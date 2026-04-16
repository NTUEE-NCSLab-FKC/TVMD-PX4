#!/usr/bin/env python3
"""
PFA Attitude Tracking Task (pymavlink)
=======================================
Based on pfa_pos_control + pfa_att_control for PFA (Partially Fully Actuated) vehicles.

Task:
  - Hover at altitude 1 m
  - Maintain pitch = 45 degrees

Control architecture (mirrors pfa_pos_control logic):
  Phase 1 – Takeoff via position control (pfa_pos_control OFFBOARD mode):
    SET_POSITION_TARGET_LOCAL_NED → trajectory_setpoint →
    pfa_pos_control (6DOF PD) → vehicle_attitude_setpoint (thrust_body + euler_attitude_des)

  Phase 2 – Stable hover verification at 1 m (pitch = 0)

  Phase 3 – Attitude tracking (SET_ATTITUDE_TARGET):
    Pitch ramps from 0° to 45° over PITCH_RAMP_TIME seconds.
    Thrust is recomputed every cycle using the same gravity-compensation formula
    as pfa_pos_control's pose_controller_6dof, plus a PD altitude hold:

        T_base  = (m * g / F_max) / cos(pitch)   # gravity compensation in body Z
        T_corr  = Kp * alt_err - Kd * vz         # altitude PD feedback (NED: vz<0 = rising)
        T_total = clamp(T_base + T_corr, 0.1, 1.0)

PFA vehicle parameters (from pfa_pos_control_params.c):
  PFA_VEH_MASS = 1.400 kg
  PFA_MAX_THR  = 24.00 N
  → normalised hover thrust ≈ 0.572

Connection: UDP to PX4 SITL – udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 1.0               # Target altitude (metres, positive up)
TARGET_PITCH_DEG  = 45.0              # Target pitch angle (degrees, nose-up positive)
TARGET_PITCH_RAD  = math.radians(TARGET_PITCH_DEG)
TARGET_ROLL_RAD   = 0.0
TARGET_YAW_RAD    = 0.0

HOVER_PHASE_DUR   = 8.0               # Seconds to hold level hover before tilting
PITCH_RAMP_TIME   = 4.0               # Seconds to ramp pitch from 0 → 45°
TRACK_PHASE_DUR   = 30.0              # Seconds to hold pitch=45° attitude tracking

# PFA controller parameters (match pfa_pos_control_params.c defaults)
PFA_MASS      = 1.400                 # kg  (PFA_VEH_MASS)
PFA_MAX_THR   = 24.00                 # N   (PFA_MAX_THR)
GRAVITY       = 9.81                  # m/s²

# Normalised hover thrust at level flight (mirrors pfa_pos_control gravity compensation)
HOVER_THR_NORM = (PFA_MASS * GRAVITY) / PFA_MAX_THR   # ≈ 0.572

# Altitude-hold PD gains for Phase 3
#   alt_err > 0  → vehicle is too low → increase thrust
#   vz < 0       → vehicle is ascending (NED convention)
ALT_KP            = 0.45              # Proportional gain on altitude error
ALT_KD            = 0.30              # Derivative gain on vertical velocity
ALT_MAX_CORR      = 0.30              # Maximum thrust correction magnitude

# Reaching / stability tolerances
ALT_TOL           = 0.15             # metres
HOVER_STABLE_TIME = 3.0              # Seconds within tolerance to confirm hover

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting to PX4 (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")
print(f"  Target: alt={TARGET_ALT_M} m, pitch={TARGET_PITCH_DEG}°")


# ─────────────────────────────────────────────────────────────
# Utility functions
# ─────────────────────────────────────────────────────────────

def euler_to_quaternion(roll, pitch, yaw):
    """
    ZYX Euler → quaternion [w, x, y, z] (MAVLink SET_ATTITUDE_TARGET order).
    Positive pitch = nose up (FRD body / NED world convention).
    """
    cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
    w =  cy * cp * cr + sy * sp * sr
    x =  cy * cp * sr - sy * sp * cr
    y =  cy * sp * cr + sy * cp * sr
    z =  sy * cp * cr - cy * sp * sr
    return [w, x, y, z]


def set_position_target(x, y, z):
    """
    SET_POSITION_TARGET_LOCAL_NED – position only, NED frame (z < 0 = up).
    Goes through pfa_pos_control's 6DOF PD controller on the PX4 side.
    type_mask = 0b0000_1111_1111_1000 → use position, ignore vel/acc/yaw/yaw_rate
    """
    master.mav.set_position_target_local_ned_send(
        0,                                          # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,                         # type_mask
        x, y, z,                                    # position (NED)
        0, 0, 0,                                    # velocity (ignored)
        0, 0, 0,                                    # acceleration (ignored)
        0, 0,                                       # yaw, yaw_rate (ignored)
    )


def set_attitude_target(roll, pitch, yaw, thrust):
    """
    SET_ATTITUDE_TARGET – attitude quaternion + normalised thrust.
    type_mask = 0b0000_0111 → ignore body rates, use attitude + thrust.

    Thrust formula (mirrors pfa_pos_control's gravity resistance):
        T = (m*g / F_max) / cos(pitch) + altitude_PD_correction
    """
    q = euler_to_quaternion(roll, pitch, yaw)
    # type_mask bit definitions:
    #   bit 0: ignore body roll  rate
    #   bit 1: ignore body pitch rate
    #   bit 2: ignore body yaw   rate
    #   bit 6: ignore thrust  (0 = USE thrust)
    #   bit 7: ignore attitude (0 = USE attitude)
    type_mask = 0b00000111      # ignore rates; use attitude + thrust
    master.mav.set_attitude_target_send(
        0,
        master.target_system,
        master.target_component,
        type_mask,
        q,                      # [w, x, y, z]
        0.0, 0.0, 0.0,          # body roll/pitch/yaw rate (ignored)
        thrust,                 # normalised throttle [0, 1]
    )


def get_local_position():
    """Returns (x, y, z, vx, vy, vz) in NED, or all-None on timeout."""
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg:
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
    return None, None, None, None, None, None


def get_attitude():
    """Returns (roll, pitch, yaw) in radians, or all-None on timeout."""
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        return msg.roll, msg.pitch, msg.yaw
    return None, None, None


def wait_for_mode(target_main_mode, timeout=5):
    """Wait until PX4 custom main mode equals target_main_mode."""
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and ((msg.custom_mode >> 16) & 0xFF) == target_main_mode:
            return True
    return False


def set_mode_offboard():
    """Request OFFBOARD mode (PX4 custom main mode = 6)."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )


def set_mode_land():
    """Request AUTO.LAND mode (PX4 main=4, sub=6)."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0,
    )


def arm(force=False):
    """Send arm command. force=True bypasses pre-flight checks."""
    p2 = 21196.0 if force else 0.0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, p2, 0.0, 0.0, 0.0, 0.0, 0.0,
    )


def compute_thrust(pitch_rad, alt_err, vz):
    """
    Compute normalised thrust for attitude-tracking phase.

    Mirrors pfa_pos_control's gravity resistance in body frame:
        control_acc  = err_acc + resist_gravity + Kv*err_vel + Kp*err_pos
        control_force = control_acc * mass
        normalized   = control_force / max_thrust
        body_thrust  = q_att.rotateVectorInverse(normalized)

    Simplified for body-Z only (pitch ≠ 0):
        T_base  = (m*g / F_max) / cos(pitch)   →  gravity compensation
        T_corr  = Kp*alt_err - Kd*vz           →  altitude PD (NED: vz<0 = ascending)
        T_total = clamp(T_base + T_corr, 0.1, 1.0)

    Args:
        pitch_rad : current pitch angle (rad), positive = nose up
        alt_err   : TARGET_ALT_M – current_alt (positive = too low)
        vz        : vertical velocity in NED (negative = ascending)
    Returns:
        Normalised thrust in [0.1, 1.0]
    """
    # Gravity compensation: more thrust needed when pitched (body Z not vertical)
    cos_pitch = max(math.cos(pitch_rad), 0.45)   # prevent extreme values above ~63°
    t_base    = HOVER_THR_NORM / cos_pitch

    # PD altitude correction
    t_corr = ALT_KP * alt_err - ALT_KD * vz
    t_corr = max(-ALT_MAX_CORR, min(ALT_MAX_CORR, t_corr))

    return max(0.10, min(1.0, t_base + t_corr))


# ─────────────────────────────────────────────────────────────
# Pre-flight check
# ─────────────────────────────────────────────────────────────
print("\nChecking local position estimate...")
x, y, z, _, _, _ = get_local_position()
if x is None:
    print("ERROR: No LOCAL_POSITION_NED received. Is EKF2 running?")
    sys.exit(1)
print(f"  OK – position: ({x:.2f}, {y:.2f}, {z:.2f}) NED")

# ─────────────────────────────────────────────────────────────
# Phase 0 – Pre-arm: stream setpoints so PX4 accepts OFFBOARD
# ─────────────────────────────────────────────────────────────
print("\n[Phase 0] Streaming initial position setpoints (5 s required by PX4)...")
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Switch to OFFBOARD
# ─────────────────────────────────────────────────────────────
print("Requesting OFFBOARD mode...")
set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD mode confirmed.")
else:
    print("  Not confirmed – retrying...")
    for _ in range(50):
        set_position_target(0.0, 0.0, -TARGET_ALT_M)
        time.sleep(0.05)
    set_mode_offboard()
    if wait_for_mode(6):
        print("  OFFBOARD confirmed on retry.")
    else:
        print("  WARNING: OFFBOARD not confirmed; continuing anyway.")

# ─────────────────────────────────────────────────────────────
# Arm
# ─────────────────────────────────────────────────────────────
print("Arming...")
for _ in range(20):
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)
arm()

armed = False
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    print("  Normal arm failed – trying force arm...")
    arm(force=True)
    for _ in range(100):
        set_position_target(0.0, 0.0, -TARGET_ALT_M)
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.05)

if armed:
    print("  Armed.")
else:
    print("ERROR: Could not arm. Check 'commander check' in pxh shell.")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────
# Phase 1 – Position control hover at 1 m (pfa_pos_control active)
# ─────────────────────────────────────────────────────────────
print(f"\n[Phase 1] Position-control hover at {TARGET_ALT_M} m (pitch=0°, pfa_pos_control)...")
print("  → SET_POSITION_TARGET_LOCAL_NED z={:.1f}".format(-TARGET_ALT_M))
print(f"  Waiting for stable hover ({HOVER_STABLE_TIME:.0f} s within ±{ALT_TOL:.2f} m)...")

phase1_start      = time.time()
stable_since      = None
last_print        = 0.0

while True:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)

    px, py, pz, vx, vy, vz = get_local_position()
    if pz is not None:
        alt      = -pz
        alt_err  = TARGET_ALT_M - alt
        in_band  = abs(alt_err) < ALT_TOL

        if in_band:
            if stable_since is None:
                stable_since = time.time()
        else:
            stable_since = None

        now = time.time()
        if now - last_print > 1.0:
            r, p, y = get_attitude()
            p_deg = math.degrees(p) if p is not None else 0.0
            stable_s = now - stable_since if stable_since else 0.0
            print(f"  alt={alt:.2f} m (err={alt_err:+.2f})  pitch={p_deg:.1f}°  "
                  f"stable={stable_s:.1f}/{HOVER_STABLE_TIME:.0f} s")
            last_print = now

        # Exit condition: stable for HOVER_STABLE_TIME OR max 20 s elapsed
        if stable_since and (time.time() - stable_since) >= HOVER_STABLE_TIME:
            print(f"  Stable hover confirmed at {alt:.2f} m.")
            break
        if time.time() - phase1_start > 20.0:
            alt = -pz if pz else 0
            print(f"  Hover timeout – continuing from alt={alt:.2f} m.")
            break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Phase 2 – Level hover hold (HOVER_PHASE_DUR seconds)
# ─────────────────────────────────────────────────────────────
print(f"\n[Phase 2] Holding level hover for {HOVER_PHASE_DUR:.0f} s...")
phase2_end = time.time() + HOVER_PHASE_DUR
last_print = 0.0
while time.time() < phase2_end:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        px, py, pz, _, _, _ = get_local_position()
        r, p, y = get_attitude()
        alt   = -pz if pz is not None else 0.0
        p_deg = math.degrees(p) if p is not None else 0.0
        print(f"  alt={alt:.2f} m  pitch={p_deg:.1f}°  "
              f"remaining={phase2_end - now:.1f} s")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Phase 3 – Attitude tracking: pitch → 45°, altitude hold
# ─────────────────────────────────────────────────────────────
print(f"\n[Phase 3] Attitude tracking: ramping pitch 0° → {TARGET_PITCH_DEG:.0f}° "
      f"over {PITCH_RAMP_TIME:.0f} s, then holding {TRACK_PHASE_DUR:.0f} s.")
print( "  → SET_ATTITUDE_TARGET with altitude-feedback thrust (mirrors pfa_pos_control)")
print(f"  Base hover thrust (level): {HOVER_THR_NORM:.3f}")
print(f"  Base hover thrust (45°) :  {HOVER_THR_NORM / math.cos(TARGET_PITCH_RAD):.3f}")
print()
print(f"  {'Time':>6}  {'Alt':>6}  {'AltErr':>7}  "
      f"{'PitchCmd':>9}  {'PitchNow':>9}  {'Thrust':>7}")
print("  " + "-"*58)

track_start = time.time()
last_print  = 0.0

while True:
    elapsed = time.time() - track_start

    # Smooth pitch ramp: linear from 0 to TARGET_PITCH_RAD over PITCH_RAMP_TIME
    ramp  = min(elapsed / PITCH_RAMP_TIME, 1.0)
    pitch_cmd = TARGET_PITCH_RAD * ramp

    # Read current state
    px, py, pz, vx, vy, vz_ned = get_local_position()
    roll_now, pitch_now, yaw_now = get_attitude()

    alt        = -pz if pz is not None else TARGET_ALT_M
    alt_err    = TARGET_ALT_M - alt
    vz         = vz_ned if vz_ned is not None else 0.0
    pitch_fb   = pitch_now if pitch_now is not None else pitch_cmd

    # Compute thrust using actual pitch for gravity compensation
    thrust = compute_thrust(pitch_fb, alt_err, vz)

    # Send attitude + thrust setpoint
    set_attitude_target(TARGET_ROLL_RAD, pitch_cmd, TARGET_YAW_RAD, thrust)

    now = time.time()
    if now - last_print >= 0.5:
        p_cmd_deg = math.degrees(pitch_cmd)
        p_now_deg = math.degrees(pitch_now) if pitch_now is not None else 0.0
        print(f"  {elapsed:6.1f}s  {alt:6.2f}m  {alt_err:+7.2f}m  "
              f"{p_cmd_deg:8.1f}°  {p_now_deg:8.1f}°  {thrust:7.3f}")
        last_print = now

    if elapsed >= TRACK_PHASE_DUR:
        break

    time.sleep(0.05)

print("\n  Attitude tracking complete.")
print(f"  Final state: alt={alt:.2f} m, pitch_cmd={math.degrees(pitch_cmd):.1f}°, "
      f"pitch_now={math.degrees(pitch_now) if pitch_now else 0:.1f}°")

# ─────────────────────────────────────────────────────────────
# Land
# ─────────────────────────────────────────────────────────────
print("\nLanding (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
