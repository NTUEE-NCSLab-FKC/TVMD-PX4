#!/usr/bin/env python3
"""
PFA Roll / Pitch Step Response Test (pymavlink)
================================================
Based on pfa_roll_tracking.py / pfa_attitude_tracking.py

Task: 懸停於 TARGET_ALT，對 roll 或 pitch 下達步階指令，量測步階響應

控制架構（與 tracking 腳本相同）：
  Python → PARAM_SET PFA_DES_ROLL / PFA_DES_PITCH（一次跳到目標值，不做 ramp）
  PX4:  pfa_pos_control → vehicle_attitude_setpoint.roll_body / pitch_body
        pfa_att_control → PD 幾何控制器驅動機體

步階序列（每輪）：0 → +AMP → 0 → -AMP → 0，每段保持 HOLD 秒

輸出：
  - 每個步階的 上升時間(10–90%)、超越量、穩定時間(±SETTLE_BAND)、穩態誤差
  - CSV：時間、指令角、實際 roll/pitch/yaw、角速度、高度

注意：
  - PARAM_SET 經 MAVLink 傳送，指令生效時間含通訊延遲（通常 < 50 ms）。
    需要精確的指令時間請以 ulog 中 vehicle_attitude_setpoint 為準。
  - 建議先綁繩 / 測試架上以小幅度（5–10°）測試。
  - XY 推力飽和角約 ±31.6°（見 pfa_roll_tracking.py），AMP 請遠小於此值。

Usage:
  python3 pfa_attitude_step_test.py --axis roll --amp 10
  python3 pfa_attitude_step_test.py --axis pitch --amp 8 --hold 6 --reps 2
"""

import argparse
import csv
import math
import sys
import time
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Arguments
# ─────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description='PFA roll/pitch step response test')
parser.add_argument('--axis', choices=['roll', 'pitch'], default='roll')
parser.add_argument('--amp', type=float, default=10.0, help='step amplitude (deg)')
parser.add_argument('--hold', type=float, default=5.0, help='hold time per step (s)')
parser.add_argument('--reps', type=int, default=2, help='number of +/- step cycles')
parser.add_argument('--alt', type=float, default=1.0, help='hover altitude (m)')
parser.add_argument('--settle-band', type=float, default=1.0, help='settling band (deg)')
parser.add_argument('--conn', default='udp:127.0.0.1:14550')
parser.add_argument('--csv', default=None, help='output csv (default: step_<axis>_<time>.csv)')
parser.add_argument('--no-land', action='store_true', help='stay in offboard hover at the end')
args = parser.parse_args()

AXIS_PARAM = 'PFA_DES_ROLL' if args.axis == 'roll' else 'PFA_DES_PITCH'
OTHER_PARAM = 'PFA_DES_PITCH' if args.axis == 'roll' else 'PFA_DES_ROLL'
CSV_PATH = args.csv or time.strftime(f'step_{args.axis}_%Y%m%d_%H%M%S.csv')

ALT_TOL = 0.15
HOVER_STABLE_TIME = 3.0
SAT_LIMIT_DEG = math.degrees(math.asin(0.3 / ((1.4 * 9.81) / 24.0)))  # ≈ 31.6°

if abs(args.amp) >= SAT_LIMIT_DEG:
    print(f"ERROR: amp={args.amp}° exceeds XY thrust saturation (~{SAT_LIMIT_DEG:.1f}°)")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print(f"Connecting to PX4 ({args.conn})...")
master = mavutil.mavlink_connection(args.conn)
master.wait_heartbeat()
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")


# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────
class State:
    """Latest telemetry, updated from every received message (nothing is discarded)."""
    roll = pitch = yaw = None
    rollspeed = pitchspeed = yawspeed = None
    z = None
    armed = False
    main_mode = None
    param_acks = {}


state = State()


def pump():
    """Drain all pending MAVLink messages into `state`."""
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            return
        t = msg.get_type()
        if t == 'ATTITUDE':
            state.roll, state.pitch, state.yaw = msg.roll, msg.pitch, msg.yaw
            state.rollspeed, state.pitchspeed, state.yawspeed = msg.rollspeed, msg.pitchspeed, msg.yawspeed
        elif t == 'LOCAL_POSITION_NED':
            state.z = msg.z
        elif t == 'HEARTBEAT' and msg.get_srcSystem() == master.target_system and msg.get_srcComponent() == 1:
            state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state.main_mode = (msg.custom_mode >> 16) & 0xFF
        elif t == 'PARAM_VALUE':
            state.param_acks[msg.param_id.rstrip('\x00')] = msg.param_value


def set_message_interval(msg_id, hz):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(msg_id), 1e6 / hz, 0, 0, 0, 0, 0)


def set_position_target(x, y, z):
    """SET_POSITION_TARGET_LOCAL_NED, position + yaw=0 (有限值，防 NaN)."""
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000,
        x, y, z, 0, 0, 0, 0, 0, 0, 0.0, 0)


def send_param(name, value):
    state.param_acks.pop(name, None)
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def keep_alive(duration, on_tick=None):
    """Stream offboard setpoints at 20 Hz for `duration` s while pumping telemetry."""
    end = time.time() + duration
    while time.time() < end:
        set_position_target(0.0, 0.0, -args.alt)
        pump()
        if on_tick:
            on_tick()
        time.sleep(0.05)


def param_set_blocking(name, value, retries=5):
    """PARAM_SET with ack, while still streaming setpoints (offboard needs > 2 Hz)."""
    for _ in range(retries):
        send_param(name, value)
        end = time.time() + 1.0
        while time.time() < end:
            keep_alive(0.05)
            v = state.param_acks.get(name)
            if v is not None and abs(v - value) < 1e-3:
                return True
    return False


def command_mode(main, sub=0.0):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(main), float(sub), 0, 0, 0, 0)


def arm(force=False):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0 if force else 0.0, 0, 0, 0, 0, 0)


def axis_angle_deg():
    a = state.roll if args.axis == 'roll' else state.pitch
    return math.degrees(a) if a is not None else None


# ─────────────────────────────────────────────────────────────
# Step 0 – 參數初始化 & 提高遙測頻率
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Requesting ATTITUDE @100 Hz, LOCAL_POSITION_NED @50 Hz...")
set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100)
set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)

for p in (AXIS_PARAM, OTHER_PARAM):
    print(f"  Setting {p} = 0° ... ", end='', flush=True)
    print("OK" if param_set_blocking(p, 0.0) else "FAILED")

keep_alive(1.0)
if state.z is None:
    print("ERROR: No LOCAL_POSITION_NED.")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────
# Step 1 – Stream setpoints → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
print("\n[Step 1] Streaming setpoints, OFFBOARD, arming...")
keep_alive(3.0)
command_mode(6.0)
keep_alive(1.0)
if state.main_mode != 6:
    command_mode(6.0)
    keep_alive(2.0)
print("  OFFBOARD confirmed." if state.main_mode == 6 else "  WARNING: OFFBOARD not confirmed.")

arm()
keep_alive(2.0)
if not state.armed:
    arm(force=True)
    keep_alive(2.0)
if not state.armed:
    print("ERROR: Failed to arm.")
    sys.exit(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 2 – 懸停穩定
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Waiting for stable hover at {args.alt} m...")
start = time.time()
stable_since = None
while time.time() - start < 25.0:
    keep_alive(0.05)
    if state.z is None:
        continue
    ok = abs(args.alt + state.z) < ALT_TOL
    stable_since = (stable_since or time.time()) if ok else None
    if stable_since and time.time() - stable_since >= HOVER_STABLE_TIME:
        break
print(f"  alt={-state.z:.2f} m, {args.axis}={axis_angle_deg():.1f}° – starting step test.")
keep_alive(2.0)

# ─────────────────────────────────────────────────────────────
# Step 3 – 步階序列
# ─────────────────────────────────────────────────────────────
sequence = []
for _ in range(args.reps):
    sequence += [args.amp, 0.0, -args.amp, 0.0]

print(f"\n[Step 3] {args.axis} steps: {sequence} (hold {args.hold:.1f} s each)")
t0 = time.time()
cmd_deg = 0.0
samples = []   # (t, cmd, roll, pitch, yaw, p, q, r, alt)
steps = []     # (t_step, from, to)
last_att = [None]


def record():
    if state.roll is None:
        return
    key = (state.roll, state.pitch, state.rollspeed)
    if key == last_att[0]:
        return  # only log new ATTITUDE samples
    last_att[0] = key
    samples.append((time.time() - t0, cmd_deg,
                    math.degrees(state.roll), math.degrees(state.pitch), math.degrees(state.yaw),
                    math.degrees(state.rollspeed), math.degrees(state.pitchspeed),
                    math.degrees(state.yawspeed), -state.z if state.z is not None else float('nan')))


keep_alive(1.0, record)  # pre-step baseline
for target in sequence:
    steps.append((time.time() - t0, cmd_deg, target))
    cmd_deg = target
    send_param(AXIS_PARAM, target)
    keep_alive(args.hold, record)
    if abs(state.param_acks.get(AXIS_PARAM, float('nan')) - target) > 1e-3:
        print(f"  WARNING: no PARAM_VALUE ack for {AXIS_PARAM}={target}")
    print(f"  cmd={target:+6.1f}°  now={axis_angle_deg():+6.1f}°  alt={-state.z:.2f} m")

# ─────────────────────────────────────────────────────────────
# Step 4 – 歸零並降落
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 4] Resetting {AXIS_PARAM} to 0°...")
param_set_blocking(AXIS_PARAM, 0.0)
keep_alive(1.0)
if not args.no_land:
    print("Landing (AUTO.LAND)...")
    command_mode(4.0, 6.0)

# ─────────────────────────────────────────────────────────────
# Save & analyse
# ─────────────────────────────────────────────────────────────
with open(CSV_PATH, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', f'{args.axis}_cmd_deg', 'roll_deg', 'pitch_deg', 'yaw_deg',
                'p_dps', 'q_dps', 'r_dps', 'alt_m'])
    w.writerows(samples)
print(f"\nSaved {len(samples)} samples to {CSV_PATH}")

col = 2 if args.axis == 'roll' else 3
print(f"\n{'step':>14}  {'rise10-90':>9}  {'overshoot':>9}  {'settle':>7}  {'ss_err':>7}")
print("  " + "─" * 54)
for i, (ts, a_from, a_to) in enumerate(steps):
    te = steps[i + 1][0] if i + 1 < len(steps) else ts + args.hold
    seg = [(s[0] - ts, s[col]) for s in samples if ts <= s[0] < te]
    if len(seg) < 5 or abs(a_to - a_from) < 1e-6:
        continue
    y0 = seg[0][1]
    dy = a_to - y0
    norm = [(t, (y - y0) / dy) for t, y in seg]      # 0 → 1 normalised response
    t10 = next((t for t, n in norm if n >= 0.1), None)
    t90 = next((t for t, n in norm if n >= 0.9), None)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    overshoot = max(0.0, max(n for _, n in norm) - 1.0) * 100.0
    outside = [t for t, y in seg if abs(y - a_to) > args.settle_band]
    settle = outside[-1] if outside else 0.0
    settled = not outside or outside[-1] < seg[-1][0] - 0.5
    tail = [y for t, y in seg if t > seg[-1][0] - 1.0]
    ss_err = a_to - sum(tail) / len(tail)
    print(f"  {a_from:+5.1f}→{a_to:+5.1f}°  "
          f"{(f'{rise:.2f}s' if rise is not None else 'n/a'):>9}  "
          f"{overshoot:8.1f}%  "
          f"{(f'{settle:.2f}s' if settled else '>hold'):>7}  "
          f"{ss_err:+6.2f}°")

print("\nDone.")
