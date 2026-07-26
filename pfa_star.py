#!/usr/bin/env python3
"""
PFA Five-Pointed Star Path Tracking (pymavlink)
=================================================
Task: 繞五角星軌跡飛行，roll=pitch=0°，yaw 朝向當前段落前進方向

五角星幾何（以懸停位置為中心，NED）：
  外接圓半徑 R = STAR_RADIUS_M
  5 個頂點（從正北起，順時針 72° 間隔）：
    v[k] = (cx + R·sin(k·2π/5),
             cy + R·cos(k·2π/5))    k = 0..4
    (NED: x=North, y=East)

    v[0] = North    v[1] = NE    v[2] = SE
    v[3] = SW       v[4] = NW

  Skip-one 順序描繪五角星（每次跨越 2 頂點）：
    0 → 2 → 4 → 1 → 3 → 0  (5 段，每段長 = 2R·sin 72° ≈ 1.902R)

  各頂點偏航轉角：全部 = 144° (五角星幾何性質)

  頂點停留期間 (VERTEX_DWELL_S)：偏航命令切換為下一段方向，
  讓 PX4 完成 144° 的偏航旋轉後再出發。

Connection: udp:127.0.0.1:14550
"""

import time
import sys
import math
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M    = 5.0    # 飛行高度 (m)
STAR_RADIUS_M   = 12.0   # 五角星外接圓半徑 (m)
SEG_SPEED_MPS   = 1.5    # 沿星邊飛行速度 (m/s)
VERTEX_DWELL_S  = 2.0    # 各頂點停留時間 (s)；PX4 需要約 1.6 s 完成 144° 偏航
NUM_LAPS        = 2      # 完整五角星飛行圈數
YAW_TRACK_PATH  = True   # True: yaw 追蹤當前段方向; False: 固定 yaw=0° (NED North)

HOVER_PHASE_DUR   = 8.0
ALT_TOL           = 0.15
HOVER_STABLE_TIME = 3.0

# ─────────────────────────────────────────────────────────────
# Derived: star geometry
# ─────────────────────────────────────────────────────────────
_STAR_VISIT  = [0, 2, 4, 1, 3]
_SEGMENTS    = [(_STAR_VISIT[i], _STAR_VISIT[(i+1) % 5]) for i in range(5)]
_SEG_LEN     = 2.0 * STAR_RADIUS_M * math.sin(math.radians(72.0))
_SEG_DUR_S   = _SEG_LEN / SEG_SPEED_MPS
_LAP_DUR_S   = 5 * (_SEG_DUR_S + VERTEX_DWELL_S)
_TOTAL_DUR_S = NUM_LAPS * _LAP_DUR_S

_LABEL = ['N', 'NE', 'SE', 'SW', 'NW']

# ─────────────────────────────────────────────────────────────
# Connect
# ─────────────────────────────────────────────────────────────
print("Connecting (udp:127.0.0.1:14550)...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"  Connected – system {master.target_system}")
print(f"  Five-pointed star mission:")
print(f"    Alt           : {TARGET_ALT_M:.0f} m")
print(f"    Outer radius  : {STAR_RADIUS_M:.1f} m")
print(f"    Segment length: {_SEG_LEN:.2f} m  (2R·sin72°)")
print(f"    Seg speed     : {SEG_SPEED_MPS:.1f} m/s  → {_SEG_DUR_S:.1f} s/seg")
print(f"    Vertex dwell  : {VERTEX_DWELL_S:.1f} s  (yaw turn 144°)")
print(f"    Laps          : {NUM_LAPS}  (≈ {_LAP_DUR_S:.0f} s/lap × {NUM_LAPS} = {_TOTAL_DUR_S:.0f} s)")
print(f"    Yaw mode      : {'追蹤前進方向' if YAW_TRACK_PATH else '固定 0° (NED North)'}")
print(f"    Tracing order : v[0(N)] → v[2(SE)] → v[4(NW)] → v[1(NE)] → v[3(SW)] → v[0(N)]")


# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────

def param_set(name, value, retries=5):
    nb = name.encode('utf-8')
    for _ in range(retries):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            nb, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if ack and ack.param_id.rstrip('\x00') == name:
            return True
        time.sleep(0.3)
    return False


def set_position_target(x, y, z, yaw_rad=0.0):
    """NED position + yaw setpoint (ignore vel/acc/yaw_rate)."""
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        float(yaw_rad), 0)


def get_local_position():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg:
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
    return None, None, None, None, None, None


def get_attitude():
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        return msg.roll, msg.pitch, msg.yaw
    return None, None, None


def wait_for_mode(target_main_mode, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and ((msg.custom_mode >> 16) & 0xFF) == target_main_mode:
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
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def arm(force=False):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0 if force else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def build_verts(cx, cy, R):
    """5 outer vertices of pentagram in NED, from North tip clockwise (72° spacing).
    v[k] = (cx + R*sin(k*2π/5), cy + R*cos(k*2π/5))
    v[0]=N, v[1]=NE, v[2]=SE, v[3]=SW, v[4]=NW
    """
    return [(cx + R * math.sin(k * 2.0 * math.pi / 5),
             cy + R * math.cos(k * 2.0 * math.pi / 5))
            for k in range(5)]


def seg_yaw(verts, src_i, dst_i):
    """NED yaw (0=North, π/2=East) pointing from src to dst vertex."""
    dx = verts[dst_i][0] - verts[src_i][0]   # North diff
    dy = verts[dst_i][1] - verts[src_i][1]   # East diff
    return math.atan2(dy, dx)


# ─────────────────────────────────────────────────────────────
# Step 0 – PFA attitude parameters
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0°...")
print("  PFA_DES_ROLL  = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED")
print("  PFA_DES_PITCH = 0° ... ", end='', flush=True)
print("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED")

# ─────────────────────────────────────────────────────────────
# Step 1 – Pre-flight → OFFBOARD → Arm
# ─────────────────────────────────────────────────────────────
print("\n[Step 1] Pre-flight → OFFBOARD → Arm...")
x0, y0, z0, _, _, _ = get_local_position()
if x0 is None:
    print("  ERROR: No LOCAL_POSITION_NED.")
    sys.exit(1)
print(f"  NED position: ({x0:.2f}, {y0:.2f}, {z0:.2f})")

print(f"  Streaming setpoints (5 s, z={-TARGET_ALT_M:.1f})...")
for _ in range(100):
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    time.sleep(0.05)

set_mode_offboard()
if wait_for_mode(6):
    print("  OFFBOARD confirmed.")
else:
    for _ in range(50):
        set_position_target(0.0, 0.0, -TARGET_ALT_M)
        time.sleep(0.05)
    set_mode_offboard()
    print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: not confirmed.")

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
    arm(force=True)
    for _ in range(100):
        set_position_target(0.0, 0.0, -TARGET_ALT_M)
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.05)

if not armed:
    print("  ERROR: Failed to arm.")
    sys.exit(1)
print("  Armed.")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stable hover at TARGET_ALT_M
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 2] Stable hover at {TARGET_ALT_M} m...")
phase_start  = time.time()
stable_since = None
last_print   = 0.0

while True:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    _, _, pz, _, _, _ = get_local_position()
    if pz is not None:
        alt     = -pz
        alt_err = TARGET_ALT_M - alt
        stable_since = (stable_since or time.time()) if abs(alt_err) < ALT_TOL else None
        now = time.time()
        if now - last_print > 1.0:
            s = now - stable_since if stable_since else 0.0
            print(f"  alt={alt:.2f} m (err={alt_err:+.2f})  "
                  f"stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
            last_print = now
        if stable_since and (time.time() - stable_since) >= HOVER_STABLE_TIME:
            print(f"  Stable at {alt:.2f} m.")
            break
        if time.time() - phase_start > 25.0:
            print(f"  Timeout – alt={alt:.2f} m, continuing.")
            break
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 3 – Hover hold; record star centre
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 3] Hover hold for {HOVER_PHASE_DUR:.0f} s (recording star centre)...")
end_t      = time.time() + HOVER_PHASE_DUR
last_print = 0.0
while time.time() < end_t:
    set_position_target(0.0, 0.0, -TARGET_ALT_M)
    now = time.time()
    if now - last_print > 2.0:
        _, _, pz, _, _, _ = get_local_position()
        print(f"  alt={-pz:.2f} m  remaining={end_t-now:.1f} s")
        last_print = now
    time.sleep(0.05)

px0, py0, _, _, _, _ = get_local_position()
cx = px0 if px0 is not None else 0.0
cy = py0 if py0 is not None else 0.0
verts = build_verts(cx, cy, STAR_RADIUS_M)

print(f"  Star centre: NED ({cx:.2f}, {cy:.2f}) m")
print(f"  5 vertices:")
for k, (vx, vy) in enumerate(verts):
    print(f"    v[{k}] {_LABEL[k]:3s}: NED ({vx:.2f}, {vy:.2f}) m")

# ─────────────────────────────────────────────────────────────
# Step 4 – Approach v[0] (North tip) from centre
# ─────────────────────────────────────────────────────────────
v0x, v0y = verts[0]
APPROACH_DUR_S = 4.0
depart_yaw = seg_yaw(verts, *_SEGMENTS[0]) if YAW_TRACK_PATH else 0.0

print(f"\n[Step 4] Approaching v[0] (North tip) in {APPROACH_DUR_S:.0f} s "
      f"(depart yaw={math.degrees(depart_yaw):.0f}°)...")
t_approach = time.time()
last_print  = 0.0

while True:
    now     = time.time()
    elapsed = now - t_approach
    frac    = min(elapsed / APPROACH_DUR_S, 1.0)
    s       = frac * frac * (3.0 - 2.0 * frac)   # cubic ease-in-out
    x_cmd   = cx + s * (v0x - cx)
    y_cmd   = cy + s * (v0y - cy)
    set_position_target(x_cmd, y_cmd, -TARGET_ALT_M, yaw_rad=depart_yaw)
    if now - last_print > 1.0:
        px, py, pz, _, _, _ = get_local_position()
        print(f"  frac={frac*100:.0f}%  NED_cmd=({x_cmd:.1f}, {y_cmd:.1f})  "
              f"NED_now=({px:.1f}, {py:.1f})  alt={-pz:.2f} m")
        last_print = now
    if frac >= 1.0:
        break
    time.sleep(0.05)

print(f"  At v[0] ({v0x:.1f}, {v0y:.1f}) m.")

# ─────────────────────────────────────────────────────────────
# Step 5 – Five-pointed star trajectory
# ─────────────────────────────────────────────────────────────
print(f"\n[Step 5] Star trajectory: {NUM_LAPS} lap(s) × 5 segs × "
      f"({_SEG_DUR_S:.1f}+{VERTEX_DWELL_S:.0f}) s ≈ {_TOTAL_DUR_S:.0f} s total")

for lap in range(NUM_LAPS):
    print(f"\n  ══ Lap {lap+1}/{NUM_LAPS} ══")

    for seg_idx, (src_i, dst_i) in enumerate(_SEGMENTS):
        src_x, src_y = verts[src_i]
        dst_x, dst_y = verts[dst_i]
        dx      = dst_x - src_x
        dy      = dst_y - src_y
        s_yaw   = seg_yaw(verts, src_i, dst_i) if YAW_TRACK_PATH else 0.0

        next_src_i, next_dst_i = _SEGMENTS[(seg_idx + 1) % 5]
        n_yaw = seg_yaw(verts, next_src_i, next_dst_i) if YAW_TRACK_PATH else 0.0

        print(f"\n  ── Seg {seg_idx+1}/5  "
              f"v[{src_i}]({_LABEL[src_i]}) → v[{dst_i}]({_LABEL[dst_i]})  "
              f"yaw={math.degrees(s_yaw):.0f}°  len={_SEG_LEN:.1f} m  "
              f"dur={_SEG_DUR_S:.1f} s ──")
        print(f"  {'Elapsed':>8}  {'Frac':>5}  {'Xcmd':>7}  {'Ycmd':>7}  "
              f"{'Xnow':>7}  {'Ynow':>7}  {'Alt':>5}")
        print("  " + "─" * 56)

        t_seg      = time.time()
        last_print = 0.0

        while True:
            now     = time.time()
            elapsed = now - t_seg
            frac    = min(elapsed / _SEG_DUR_S, 1.0)
            x_cmd   = src_x + frac * dx
            y_cmd   = src_y + frac * dy

            set_position_target(x_cmd, y_cmd, -TARGET_ALT_M, yaw_rad=s_yaw)

            if now - last_print > 1.0:
                px, py, pz, _, _, _ = get_local_position()
                print(f"  {elapsed:8.1f}s  {frac:5.2f}  "
                      f"{x_cmd:7.2f}m  {y_cmd:7.2f}m  "
                      f"{px:7.2f}m  {py:7.2f}m  {-pz:5.2f}m")
                last_print = now

            if frac >= 1.0:
                break
            time.sleep(0.05)

        # Dwell at vertex – rotate yaw to next segment direction
        print(f"  v[{dst_i}] reached. Dwell {VERTEX_DWELL_S:.0f} s, "
              f"yaw → {math.degrees(n_yaw):.0f}° (Δ=144°)")
        t_dwell    = time.time()
        last_print = 0.0
        while time.time() - t_dwell < VERTEX_DWELL_S:
            set_position_target(dst_x, dst_y, -TARGET_ALT_M, yaw_rad=n_yaw)
            time.sleep(0.05)

print(f"\n  Star trajectory complete ({NUM_LAPS} lap(s)).")

# ─────────────────────────────────────────────────────────────
# Step 6 – Return to centre and hold 5 s
# ─────────────────────────────────────────────────────────────
RETURN_DUR_S = 4.0
print(f"\n[Step 6] Returning to star centre ({cx:.1f}, {cy:.1f}) m, hold 5 s...")
t_return   = time.time()
last_print = 0.0

while True:
    now     = time.time()
    elapsed = now - t_return
    frac    = min(elapsed / RETURN_DUR_S, 1.0)
    s       = frac * frac * (3.0 - 2.0 * frac)
    x_cmd   = v0x + s * (cx - v0x)
    y_cmd   = v0y + s * (cy - v0y)
    set_position_target(x_cmd, y_cmd, -TARGET_ALT_M, yaw_rad=0.0)
    if frac >= 1.0:
        break
    time.sleep(0.05)

end_t      = time.time() + 5.0
last_print = 0.0
while time.time() < end_t:
    set_position_target(cx, cy, -TARGET_ALT_M, yaw_rad=0.0)
    now = time.time()
    if now - last_print > 1.0:
        px, py, pz, _, _, _ = get_local_position()
        roll, pitch, _ = get_attitude()
        r_deg = math.degrees(roll)  if roll  is not None else 0.0
        p_deg = math.degrees(pitch) if pitch is not None else 0.0
        print(f"  NED=({px:.2f}, {py:.2f})  alt={-pz:.2f} m  "
              f"roll={r_deg:.1f}°  pitch={p_deg:.1f}°")
        last_print = now
    time.sleep(0.05)

# ─────────────────────────────────────────────────────────────
# Step 7 – Land
# ─────────────────────────────────────────────────────────────
print("\n[Step 7] Landing (AUTO.LAND)...")
set_mode_land()
time.sleep(15)
print("Done.")
