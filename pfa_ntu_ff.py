#!/usr/bin/env python3
"""
PFA Letter Trajectory "NTU" with Feedforward (pymavlink)
=========================================================
用飛行軌跡「寫」出字母（預設 NTU），roll = pitch = 0°，yaw 全程固定為起飛航向
（TVMD 可全向平移，不需要轉機頭），同時送出位置 + 速度 + 加速度前饋。

■ 與 MAVROS 版的差異
  - 不需要 ROS：直接用 pymavlink 送 SET_POSITION_TARGET_LOCAL_NED（NED 座標，
    type_mask 只忽略 yaw_rate）。MAVROS 版的 ENU 幾何在這裡直接換成 NED。
  - 不下載參數、不做 timesync、只要求 2 個遙測串流（5 Hz）→ 經 57600 數傳也很輕。
  - 兩個背景執行緒：_rx_worker 唯一讀 socket、_sp_worker 以 SP_RATE_HZ 送 setpoint
    並每秒送 heartbeat；主執行緒的任何等待都不會中斷串流。
  - 平滑起飛（同 *_ff_smooth_mavros.py）：解鎖前 setpoint = 地面位置與航向，
    等 PX4 起飛緩升結束後以最小 jerk 曲線爬升。
  - 所有 yaw 轉向都用最小 jerk 曲線（MAVROS 版是階躍），轉完再停 YAW_SETTLE_S 才出發。
  - 路徑中心 = 懸停 setpoint（地面起飛點正上方）。
  - 若途中離開 OFFBOARD 或被上鎖：立即停止，不再切模式，交給飛手 / failsafe。

■ 字母幾何（字母平面座標 u = 右、v = 上，單位 m）
  每個字母寬 LETTER_W、高 LETTER_H，字距 LETTER_GAP，整串字以路徑中心為中心。
  每個字母由數個「筆畫」組成，筆畫由直線與圓弧串成：
    N：左豎 ↑ → 斜線 ↘ → 右豎 ↑
    T：橫線 → → 直豎 ↓（中間的移動自動補成「空移」）
    U：左豎 ↓ → 半圓（半徑 W/2）→ 右豎 ↑（直線與圓弧相切，可不停一路畫完）
  筆畫之間不相連時自動插入直線「空移」（TRANSIT_SPEED）。
  新增字母：在 LETTERS 字典裡加上筆畫即可（座標以字母左下角為原點）。

■ 速度曲線
  每一段（筆畫或空移）以等速 V 飛行，前後各 RAMP_S 秒 smoothstep 加減速
  （起停速度、加速度皆為 0），段與段之間停 CORNER_DWELL_S 秒。
  段太短（L < V·RAMP_S）時自動降速，確保仍有完整的加減速。
  前饋：v = T·ṡ，a = T·s̈ + κ·N·ṡ²（圓弧段含向心加速度 V²/r）。

■ 書寫平面 PLANE
  'horizontal'：寫在 TARGET_ALT_M 高度的水平面上（俯視可讀；字的「上」= 起飛時機頭方向）
  'vertical'  ：寫在垂直面上（站在飛機後方看可讀；字的「上」= 高度增加，
                底部高度 = ALT_BOTTOM_M，頂部 = ALT_BOTTOM_M + LETTER_H）

■ 預覽（不連線、不起飛）
  python3 pfa_ntu_ff.py --preview
  → 印出每一段的長度與時間，並輸出 ntu_path.svg（黑線 = 筆畫、灰虛線 = 空移）、ntu_path.csv

■ 連線（MAVProxy 分流時）
  mavproxy.py --master=/dev/ttyUSB0 --baud 57600 \\
              --out udpout:127.0.0.1:14550 --out udpin:0.0.0.0:14551
  預設連 udpout:127.0.0.1:14551（MAVROS 不能同時使用 14551）；也可用參數指定：
    python3 pfa_ntu_ff.py udp:127.0.0.1:14550    （QGC 不能同時佔 14550）
"""

import sys
import time
import math
import threading
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TEXT             = 'NTU'         # 要寫的字（LETTERS 裡有定義的字母）
PLANE            = 'horizontal'  # 'horizontal'：水平面；'vertical'：垂直面
TARGET_ALT_M     = 1.0     # 懸停高度 (m)；horizontal 時也是書寫高度
ALT_BOTTOM_M     = 0.6     # vertical 時字的底部高度 (m)；頂部 = ALT_BOTTOM_M + LETTER_H
LETTER_H         = 1.0     # 字高 (m)
LETTER_W         = 0.6     # 字寬 (m)
LETTER_GAP       = 0.3     # 字距 (m)
DRAW_SPEED_MPS   = 0.2     # 筆畫等速段速度 (m/s)
TRANSIT_SPEED_MPS = 0.2    # 空移等速段速度 (m/s)
RAMP_S           = 1.0     # 每段起步加速 / 結束減速時間 (s)
CORNER_DWELL_S   = 1.0     # 段與段之間的停留時間 (s)
APPROACH_PEAK_MPS = 0.3    # 中心 ↔ 起點的最小 jerk 峰值速度 (m/s)

# MAVLink 連線：可用第一個參數覆寫，例如
#   python3 pfa_ntu_ff.py udp:127.0.0.1:14550        （監聽 MAVProxy 的 udpout；QGC 不能同時佔 14550）
# 預設接 MAVProxy 的 `--out udpin:0.0.0.0:14551`（MAVROS 不能同時使用 14551）
CONNECTION        = sys.argv[1] if len(sys.argv) > 1 else 'udpout:127.0.0.1:14551'

HOVER_PHASE_DUR   = 8.0    # 起飛後定高懸停時間 (s)
ALT_TOL           = 0.15   # 穩定懸停高度容差 (m)
HOVER_STABLE_TIME = 3.0    # 穩定判定時間 (s)
ALT_WAIT_TIMEOUT  = 25.0   # 等待高度穩定的上限 (s)

# 平滑起飛（與 *_ff_smooth_mavros.py 相同）
CLIMB_PEAK_MPS    = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S       = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S  = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)
YAW_PEAK_RATE_DPS = 45.0   # 起飛後轉向起始航向的峰值角速度 (deg/s)
YAW_MIN_TURN_S    = 1.0    # 轉向最短時間 (s)

SP_RATE_HZ        = 10.0   # offboard setpoint 串流頻率；經 57600 數傳 10 Hz ≈ 650 B/s
LOG_PERIOD_S      = 0.5    # 追蹤狀態印出間隔 (s)
PX4_SPEED_XY_MAX  = 1.0    # pfa_pos_control 的 _speed_xy_max (m/s)，水平速度參考上限
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × L / T

# 遙測串流 (msg id → Hz)：LOCAL_POSITION_NED 5 Hz、ATTITUDE 5 Hz（數傳頻寬有限，維持低頻）
_STREAMS_HZ = {32: 5.0, 30: 5.0}

# ─────────────────────────────────────────────────────────────
# Letter geometry（u = 右、v = 上；每個字母以左下角為原點）
#   ('L', (u0, v0), (u1, v1))                  直線
#   ('A', (uc, vc), r, a0, a1)                 圓弧，角度 a0 → a1 (rad)，a1 > a0 為逆時針
# 一個筆畫 = 相切連續的 primitive 串列（可一路不停）；筆畫之間會停下或插入空移
# ─────────────────────────────────────────────────────────────
def _letters(W, H):
    r = W / 2.0
    return {
        'N': [[('L', (0, 0), (0, H))],
              [('L', (0, H), (W, 0))],
              [('L', (W, 0), (W, H))]],
        'T': [[('L', (0, H), (W, H))],
              [('L', (W / 2, H), (W / 2, 0))]],
        'U': [[('L', (0, H), (0, r)),
               ('A', (r, r), r, math.pi, 2 * math.pi),
               ('L', (W, r), (W, H))]],
    }


LETTERS = _letters(LETTER_W, LETTER_H)


def _prim_len(pr):
    if pr[0] == 'L':
        return math.dist(pr[1], pr[2])
    return pr[2] * abs(pr[4] - pr[3])


def _prim_eval(pr, s):
    """primitive 上弧長 s 處的 (點, 單位切線, 曲率向量)（2D，u-v 平面）。"""
    if pr[0] == 'L':
        (u0, v0), (u1, v1) = pr[1], pr[2]
        L = math.dist(pr[1], pr[2])
        tu, tv = (u1 - u0) / L, (v1 - v0) / L
        return (u0 + s * tu, v0 + s * tv), (tu, tv), (0.0, 0.0)
    (uc, vc), r, a0, a1 = pr[1], pr[2], pr[3], pr[4]
    d = 1.0 if a1 >= a0 else -1.0
    th = a0 + d * s / r
    c, sn = math.cos(th), math.sin(th)
    return (uc + r * c, vc + r * sn), (-d * sn, d * c), (-c / r, -sn / r)


class Piece:
    """一段可連續飛行的路徑（筆畫或空移），以弧長參數化。"""

    def __init__(self, prims, kind, label):
        self.prims, self.kind, self.label = prims, kind, label
        self.lens = [_prim_len(p) for p in prims]
        self.L = sum(self.lens)
        v_nom = DRAW_SPEED_MPS if kind == 'draw' else TRANSIT_SPEED_MPS
        self.V = min(v_nom, self.L / RAMP_S)          # 太短就降速，保留完整加減速
        self.path_dur = self.L / self.V
        self.T = self.path_dur + RAMP_S

    def start(self):
        return _prim_eval(self.prims[0], 0.0)[0]

    def end(self):
        return _prim_eval(self.prims[-1], self.lens[-1])[0]

    def at_s(self, s):
        s = min(max(s, 0.0), self.L)
        for pr, ln in zip(self.prims, self.lens):
            if s <= ln + 1e-9:
                return _prim_eval(pr, s)
            s -= ln
        return _prim_eval(self.prims[-1], self.lens[-1])

    def state2d(self, t):
        """t 秒時 (p, v, a)，2D。"""
        tau, td, tdd = ramp_time(t, self.path_dur, RAMP_S)
        p, tg, k = self.at_s(self.V * tau)
        sd, sdd = self.V * td, self.V * tdd
        v = (tg[0] * sd, tg[1] * sd)
        a = (tg[0] * sdd + k[0] * sd * sd, tg[1] * sdd + k[1] * sd * sd)
        return p, v, a


def ramp_time(t, path_dur, ramp):
    """時間扭曲 τ(t)：τ̇ 在前後 ramp 秒內以 smoothstep 3u²−2u³ 平滑 0↔1，中段 τ̇ = 1。"""
    total = path_dur + ramp
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= total:
        return path_dur, 0.0, 0.0
    if t < ramp:
        u = t / ramp
        return ramp * u**3 * (1.0 - 0.5 * u), u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u) / ramp
    if t > total - ramp:
        r = (total - t) / ramp
        return path_dur - ramp * r**3 * (1.0 - 0.5 * r), r * r * (3.0 - 2.0 * r), -6.0 * r * (1.0 - r) / ramp
    return t - 0.5 * ramp, 1.0, 0.0


def _shift(pr, du, dv):
    if pr[0] == 'L':
        return ('L', (pr[1][0] + du, pr[1][1] + dv), (pr[2][0] + du, pr[2][1] + dv))
    return ('A', (pr[1][0] + du, pr[1][1] + dv), pr[2], pr[3], pr[4])


def build_pieces(text):
    """把字串轉成依序飛行的 Piece 串列（含自動空移），座標以整串字的中心為原點。"""
    for ch in text:
        if ch not in LETTERS:
            sys.exit(f"  ERROR: 字母 {ch!r} 沒有定義（目前有：{''.join(sorted(LETTERS))}）")
    total_w = len(text) * LETTER_W + (len(text) - 1) * LETTER_GAP
    v_off = -LETTER_H / 2.0 if PLANE == 'horizontal' else 0.0     # vertical：v = 0 是字底
    pieces, last = [], None
    for k, ch in enumerate(text):
        du = k * (LETTER_W + LETTER_GAP) - total_w / 2.0
        for j, stroke in enumerate(LETTERS[ch]):
            prims = [_shift(p, du, v_off) for p in stroke]
            pc = Piece(prims, 'draw', f"{ch} stroke {j + 1}")
            if last is not None and math.dist(last, pc.start()) > 1e-6:
                pieces.append(Piece([('L', last, pc.start())], 'transit', f"move → {ch}"))
            pieces.append(pc)
            last = pc.end()
    return pieces


if PLANE not in ('horizontal', 'vertical'):
    sys.exit(f"  ERROR: PLANE 必須是 'horizontal' 或 'vertical'（目前 {PLANE!r}）")
PIECES      = build_pieces(TEXT)
_DRAW_LEN   = sum(p.L for p in PIECES if p.kind == 'draw')
_MOVE_LEN   = sum(p.L for p in PIECES if p.kind == 'transit')
_TOTAL_DUR  = sum(p.T for p in PIECES) + CORNER_DWELL_S * len(PIECES)
_ACC_MAX    = max(max(1.5 * p.V / RAMP_S,
                      max((p.V ** 2 / pr[2] for pr in p.prims if pr[0] == 'A'), default=0.0))
                  for p in PIECES)


def _summary():
    print(f"  Text {TEXT!r} on {PLANE} plane: {len(PIECES)} pieces, draw {_DRAW_LEN:.2f} m, "
          f"transit {_MOVE_LEN:.2f} m, ≈ {_TOTAL_DUR:.0f} s (+ approach / return)")
    print(f"    letter {LETTER_W:.2f} × {LETTER_H:.2f} m, gap {LETTER_GAP:.2f} m, "
          f"draw {DRAW_SPEED_MPS:.2f} m/s, transit {TRANSIT_SPEED_MPS:.2f} m/s, max accel ≈ {_ACC_MAX:.2f} m/s²")
    for i, p in enumerate(PIECES):
        (u0, v0), (u1, v1) = p.start(), p.end()
        print(f"    {i + 1:2d}. {p.label:14s} {p.kind:7s} L={p.L:4.2f} m  V={p.V:4.2f}  T={p.T:4.1f} s  "
              f"({u0:+.2f},{v0:+.2f}) → ({u1:+.2f},{v1:+.2f})")


def _preview():
    rows, t_acc = [], 0.0
    for i, p in enumerate(PIECES):
        n = max(2, int(p.T * 20))
        for k in range(n + 1):
            t = p.T * k / n
            (u, v), (vu, vv), _ = p.state2d(t)
            rows.append((t_acc + t, i, p.kind, u, v, math.hypot(vu, vv)))
        t_acc += p.T + CORNER_DWELL_S
    with open('ntu_path.csv', 'w') as f:
        f.write('t,piece,kind,u_right_m,v_up_m,speed_mps\n')
        for r in rows:
            f.write(f"{r[0]:.2f},{r[1]},{r[2]},{r[3]:.4f},{r[4]:.4f},{r[5]:.4f}\n")
    us = [r[3] for r in rows]; vs = [r[4] for r in rows]
    pad, sc = 0.2, 200.0
    u0, v1 = min(us) - pad, max(vs) + pad
    w, h = (max(us) - min(us) + 2 * pad) * sc, (max(vs) - min(vs) + 2 * pad) * sc
    X = lambda u: (u - u0) * sc
    Y = lambda v: (v1 - v) * sc
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" '
           f'viewBox="0 0 {w:.0f} {h:.0f}" style="background:#fff">']
    for i, p in enumerate(PIECES):
        pts = ' '.join(f"{X(r[3]):.1f},{Y(r[4]):.1f}" for r in rows if r[1] == i)
        style = ('stroke="#111" stroke-width="5"' if p.kind == 'draw'
                 else 'stroke="#999" stroke-width="2" stroke-dasharray="8 6"')
        svg.append(f'<polyline points="{pts}" fill="none" {style} stroke-linecap="round"/>')
    s0 = PIECES[0].start()
    svg.append(f'<circle cx="{X(s0[0]):.1f}" cy="{Y(s0[1]):.1f}" r="7" fill="#2a7"/>')
    svg.append(f'<circle cx="{X(0):.1f}" cy="{Y(0 if PLANE == "horizontal" else min(vs)):.1f}" r="5" fill="#c33"/>')
    svg.append('</svg>')
    with open('ntu_path.svg', 'w') as f:
        f.write('\n'.join(svg))
    print("  Wrote ntu_path.svg (black = draw, dashed = transit, green = start, red = centre) and ntu_path.csv")


_summary()
if '--preview' in sys.argv[1:]:
    _preview()
    sys.exit(0)

# ─────────────────────────────────────────────────────────────
# Connect
# udpout 連線必須先送出封包，MAVProxy 的 udpin 才知道要回傳給誰，
# 所以先送 heartbeat（MAV_TYPE_ONBOARD_CONTROLLER），之後由 _sp_worker 每秒補送。
# 只接受飛控本身的 heartbeat（排除 MAVProxy / QGC 的 GCS heartbeat）。
# ─────────────────────────────────────────────────────────────
print(f"Connecting to PX4 ({CONNECTION})...")
master = mavutil.mavlink_connection(CONNECTION, source_system=255, source_component=191)


def _send_heartbeat():
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


_t_conn = time.time()
_last_hb = 0.0
while True:
    if time.time() - _last_hb > 1.0:
        _send_heartbeat()
        _last_hb = time.time()
    _m = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
    if (_m is not None and _m.type != mavutil.mavlink.MAV_TYPE_GCS
            and _m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
        master.target_system = _m.get_srcSystem()
        break
    if time.time() - _t_conn > 30.0:
        sys.exit("  ERROR: 30 s 內沒有收到飛控 heartbeat（檢查 MAVProxy 輸出埠 / 連線字串）")
master.target_component = 1
print(f"  Connected – system {master.target_system}, component {master.target_component}")

for _msg_id, _hz in _STREAMS_HZ.items():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(_msg_id), 1e6 / _hz, 0, 0, 0, 0, 0)
    time.sleep(0.05)


# ─────────────────────────────────────────────────────────────
# Telemetry state cache（只由 _rx_worker 寫入，主執行緒只讀）
# ─────────────────────────────────────────────────────────────
_ned    = {'x': 0.0, 'y': 0.0, 'z': -0.01, 't': 0.0}            # LOCAL_POSITION_NED
_att    = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 't': 0.0}     # ATTITUDE (NED/FRD)
_hb     = {'main_mode': None, 'armed': False, 't': 0.0}          # 飛控 HEARTBEAT
_params   = {}                         # PARAM_VALUE: name → (value, recv_time)
_param_cv = threading.Condition()

_PX4_MODE_NAMES = {1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: 'AUTO',
                   5: 'ACRO', 6: 'OFFBOARD', 7: 'STABILIZED', 8: 'RATTITUDE'}


def _handle_msg(msg):
    t = msg.get_type()
    if t == 'LOCAL_POSITION_NED':
        _ned['x'], _ned['y'], _ned['z'] = msg.x, msg.y, msg.z
        _ned['t'] = time.time()
    elif t == 'ATTITUDE':
        _att['roll'], _att['pitch'], _att['yaw'] = msg.roll, msg.pitch, msg.yaw
        _att['t'] = time.time()
    elif t == 'HEARTBEAT':
        if (msg.get_srcSystem() != master.target_system
                or msg.type == mavutil.mavlink.MAV_TYPE_GCS
                or msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID):
            return
        main_mode = (msg.custom_mode >> 16) & 0xFF
        if _hb['main_mode'] is not None and main_mode != _hb['main_mode']:
            print(f"\n  [MODE] {_PX4_MODE_NAMES.get(_hb['main_mode'], _hb['main_mode'])}"
                  f" → {_PX4_MODE_NAMES.get(main_mode, main_mode)}"
                  f"  (setpoint 最大間隔 {_sp_stat['max_gap']*1000:.0f} ms)", flush=True)
        _hb['main_mode'] = main_mode
        _hb['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        _hb['t'] = time.time()
    elif t == 'PARAM_VALUE':
        with _param_cv:
            _params[msg.param_id.rstrip('\x00')] = (msg.param_value, time.time())
            _param_cv.notify_all()
    elif t == 'STATUSTEXT':
        print(f"\n  [PX4] {msg.text.rstrip(chr(0))}", flush=True)
    elif t == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"\n  [ARM] rejected (result={msg.result})，原因見上方 [PX4] 訊息", flush=True)


def _rx_worker():
    """唯一讀取 socket 的執行緒；主執行緒的任何等待都不會卡住 setpoint 串流。"""
    while _running.is_set():
        try:
            msg = master.recv_match(blocking=True, timeout=0.1)
        except Exception as e:          # 單一壞封包不應終止接收執行緒
            print(f"\n  [RX] error: {e}", flush=True)
            continue
        if msg is not None and msg.get_type() != 'BAD_DATA':
            _handle_msg(msg)


# ─────────────────────────────────────────────────────────────
# Background setpoint thread（唯一發 setpoint 的執行緒，固定 SP_RATE_HZ）
# PX4 offboard_control_mode 超過 COM_OF_LOSS_T 未更新 → offboard signal lost。
# ─────────────────────────────────────────────────────────────
_tx_lock   = threading.Lock()      # pymavlink 的 mav.send() 非 thread-safe（seq 計數）
_running   = threading.Event()
_running.set()
# 目前 setpoint (NED)：((x, y, z), (vx, vy, vz), (ax, ay, az), yaw)，單一 tuple 指派（原子）
_sp_pvay   = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
_SP_TYPE_MASK = 0b0000100000000000   # position + velocity + acceleration + yaw，忽略 yaw_rate
_sp_active = threading.Event()
_sp_active.set()
_sp_stat   = {'last': 0.0, 'max_gap': 0.0}


def _send(fn, *args):
    with _tx_lock:
        fn(*args)


def _sp_worker():
    period = 1.0 / SP_RATE_HZ
    next_t = time.monotonic()
    last_hb = 0.0
    while _running.is_set():
        now = time.monotonic()
        if now - last_hb >= 1.0:
            _send(_send_heartbeat)
            last_hb = now
        if _sp_active.is_set():
            (x, y, z), (vx, vy, vz), (ax, ay, az), yaw = _sp_pvay
            _send(master.mav.set_position_target_local_ned_send,
                  0, master.target_system, master.target_component,
                  mavutil.mavlink.MAV_FRAME_LOCAL_NED, _SP_TYPE_MASK,
                  x, y, z, vx, vy, vz, ax, ay, az, yaw, 0)
            if _sp_stat['last']:
                gap = now - _sp_stat['last']
                _sp_stat['max_gap'] = max(_sp_stat['max_gap'], gap)
                if gap > 0.3:
                    print(f"\n  [SP] WARNING: setpoint gap {gap*1000:.0f} ms", flush=True)
            _sp_stat['last'] = now
        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()


_rx_thread = threading.Thread(target=_rx_worker, daemon=True)
_sp_thread = threading.Thread(target=_sp_worker, daemon=True)
_rx_thread.start()
_sp_thread.start()


# ─────────────────────────────────────────────────────────────
# Helpers（皆不讀 socket，只讀 _rx_worker 的快取）
# ─────────────────────────────────────────────────────────────

def set_pva(p, v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0), yaw=None):
    """更新 setpoint 位置 / 速度 / 加速度 / yaw (NED)；實際發送由 _sp_worker 負責。"""
    global _sp_pvay
    _sp_pvay = (tuple(p), tuple(v), tuple(a), _sp_pvay[3] if yaw is None else yaw)


def min_jerk(t: float, T: float):
    """最小 jerk 曲線 s(t)∈[0,1]：回傳 (s, ṡ, s̈)，起訖速度與加速度皆為 0。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    u = t / T
    return (u**3 * (10.0 - 15.0 * u + 6.0 * u * u),
            30.0 * u * u * (1.0 - u) ** 2 / T,
            60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T))


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_turn_duration(yaw0: float, yaw1: float) -> float:
    d = abs(wrap_pi(yaw1 - yaw0))
    return max(YAW_MIN_TURN_S, MIN_JERK_PEAK * d / math.radians(YAW_PEAK_RATE_DPS))


def param_set(name: str, value: float, retries: int = 5, timeout: float = 1.5) -> bool:
    """PARAM_SET + 等待本次送出之後的 PARAM_VALUE，並確認數值一致。"""
    nb = name.encode('utf-8')
    for attempt in range(retries):
        t_sent = time.time()
        _send(master.mav.param_set_send, master.target_system, master.target_component,
              nb, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        with _param_cv:
            got = _param_cv.wait_for(
                lambda: name in _params and _params[name][1] >= t_sent, timeout)
            actual = _params[name][0] if got else None
        if got and abs(actual - value) < 0.01:
            return True
    return False


def param_get(name: str, timeout: float = 3.0):
    t_sent = time.time()
    _send(master.mav.param_request_read_send, master.target_system, master.target_component,
          name.encode('utf-8'), -1)
    with _param_cv:
        if _param_cv.wait_for(lambda: name in _params and _params[name][1] >= t_sent, timeout):
            return _params[name][0]
    return None


def wait_until(cond, timeout: float) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def wait_for_mode(target_main_mode: int, timeout: float = 5) -> bool:
    return wait_until(lambda: _hb['main_mode'] == target_main_mode, timeout)


def flight_ok() -> bool:
    return _hb['main_mode'] == 6 and _hb['armed']


def set_mode_offboard():
    _send(master.mav.command_long_send, master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 6.0, 0, 0, 0, 0, 0)


def set_mode_land():
    _send(master.mav.command_long_send, master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 4.0, 6.0, 0, 0, 0, 0)


def arm(force: bool = False):
    _send(master.mav.command_long_send, master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
          1.0, 21196.0 if force else 0.0, 0, 0, 0, 0, 0)


def shutdown(code: int):
    _sp_active.clear()
    _running.clear()
    sys.exit(code)


class Aborted(Exception):
    """離開 OFFBOARD / 被上鎖：交給飛手 / failsafe，不再切模式。"""


def check_flight():
    if not flight_ok():
        mode = _PX4_MODE_NAMES.get(_hb['main_mode'], _hb['main_mode'])
        raise Aborted(f"left OFFBOARD / disarmed (mode={mode}, armed={_hb['armed']})")


def run_trajectory(fn, duration, title):
    """以 fn(t) -> (p, v, a, yaw)（NED）送出 setpoint 直到 duration，定期印出追蹤誤差。"""
    print(f"  ── {title}  ({duration:.1f} s) ──")
    t0 = time.time()
    last_log = 0.0
    while True:
        check_flight()
        t = min(time.time() - t0, duration)
        p, v, a, yaw = fn(t)
        set_pva(p, v, a, yaw)
        now = time.time()
        if now - last_log >= LOG_PERIOD_S:
            xn, yn, zn = _ned['x'], _ned['y'], _ned['z']
            print(f"    {t:5.1f}s  cmd N/E=({p[0]:6.2f},{p[1]:6.2f}) alt={-p[2]:4.2f}  "
                  f"pos=({xn:6.2f},{yn:6.2f}) alt={-zn:4.2f}  "
                  f"err={math.hypot(p[0] - xn, p[1] - yn):4.2f}m  "
                  f"|v_ff|={math.hypot(v[0], v[1]):4.2f}  |a_ff|={math.hypot(a[0], a[1]):4.2f}  "
                  f"yaw={math.degrees(yaw):6.1f}° (now {math.degrees(_att['yaw']):6.1f}°)")
            last_log = now
        if t >= duration:
            return fn(duration)
        time.sleep(0.05)


def turn_and_hold(p, yaw0, yaw1, duration, title):
    """在 p 保持（零速度 / 加速度前饋），yaw 以最小 jerk 曲線從 yaw0 轉到 yaw1。"""
    d = wrap_pi(yaw1 - yaw0)

    def fn(t):
        s, _, _ = min_jerk(t, duration)
        return p, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), wrap_pi(yaw0 + s * d)
    run_trajectory(fn, duration, title)


def warn_speed(peak_xy_mps, what):
    if peak_xy_mps > PX4_SPEED_XY_MAX:
        print(f"  WARNING: {what} peak horizontal speed {peak_xy_mps:.2f} m/s > "
              f"pfa_pos_control _speed_xy_max {PX4_SPEED_XY_MAX:.1f} m/s "
              f"→ velocity feedforward will be clipped. Slow down or shrink the path.")


# ─────────────────────────────────────────────────────────────
# Trajectory building blocks（fn(t) -> (p, v, a, yaw)，NED）
# ─────────────────────────────────────────────────────────────

def line_duration(p0, p1, peak_speed, min_dur=0.0):
    """直線段時間：讓最小 jerk 曲線的峰值速度 = peak_speed（至少 min_dur）。"""
    dist = math.dist(p0, p1)
    return max(min_dur, MIN_JERK_PEAK * dist / peak_speed) if dist > 1e-6 else min_dur


def line_segment(p0, p1, T, yaw):
    """p0 → p1 的直線段（最小 jerk），yaw 固定。"""
    d = tuple(b - a for a, b in zip(p0, p1))

    def fn(t):
        s, sd, sdd = min_jerk(t, T)
        return (tuple(a + s * k for a, k in zip(p0, d)),
                tuple(sd * k for k in d),
                tuple(sdd * k for k in d),
                yaw)
    return fn


def letter_frame(cn, ce, yaw):
    """字母平面 (u, v) → NED 的原點與單位向量。u = 起飛航向的右方。"""
    e_u = (-math.sin(yaw), math.cos(yaw), 0.0)
    if PLANE == 'horizontal':
        e_v = (math.cos(yaw), math.sin(yaw), 0.0)           # 上 = 機頭方向
        origin = (cn, ce, -TARGET_ALT_M)
    else:
        e_v = (0.0, 0.0, -1.0)                              # 上 = 高度增加
        origin = (cn, ce, -ALT_BOTTOM_M)
    return origin, e_u, e_v


def to_ned(frame, uv):
    o, eu, ev = frame
    return tuple(o[i] + uv[0] * eu[i] + uv[1] * ev[i] for i in range(3))


def vec_ned(frame, w):
    _, eu, ev = frame
    return tuple(w[0] * eu[i] + w[1] * ev[i] for i in range(3))


def piece_fn(frame, piece, yaw):
    def fn(t):
        p, v, a = piece.state2d(t)
        return to_ned(frame, p), vec_ned(frame, v), vec_ned(frame, a), yaw
    return fn


warn_speed(max(DRAW_SPEED_MPS, TRANSIT_SPEED_MPS, APPROACH_PEAK_MPS), "letters")
if PLANE == 'vertical' and ALT_BOTTOM_M < 0.4:
    print("  WARNING: ALT_BOTTOM_M < 0.4 m，接近地面效應與光流最低高度，建議 ≥ 0.5 m")

# ─────────────────────────────────────────────────────────────
# Step 0 – PFA 姿態參數歸零、讀取起飛緩升參數
# ─────────────────────────────────────────────────────────────
print("\n[Step 0] PFA_DES_ROLL/PITCH = 0°")
for _name in ('PFA_DES_ROLL', 'PFA_DES_PITCH'):
    print(f"  {_name} = 0 ... {'OK' if param_set(_name, 0.0) else 'FAILED (continuing)'}")

print("\n[Step 1] Checking local position / attitude...")
if not wait_until(lambda: _ned['t'] > 0.0 and _att['t'] > 0.0, 5.0):
    print("  ERROR: No LOCAL_POSITION_NED / ATTITUDE received. Is EKF2 running?")
    shutdown(1)
ground     = (_ned['x'], _ned['y'], _ned['z'])
ground_yaw = _att['yaw']
print(f"  OK – NED position: ({ground[0]:.2f}, {ground[1]:.2f}, {ground[2]:.2f})  "
      f"yaw={math.degrees(ground_yaw):.1f}°")

tkf_byp = param_get('PFA_TKF_BYP')
spool_s = param_get('COM_SPOOLUP_TIME')
ramp_s  = param_get('MPC_TKO_RAMP_T')
if tkf_byp is not None and int(round(tkf_byp)) == 0:
    print("  WARNING: PFA_TKF_BYP = 0 → OFFBOARD 下收不到 want_takeoff，推力上限會卡在 0.1，"
          "請改回 1。")
spool_s = 1.0 if spool_s is None else spool_s
ramp_s  = 3.0 if ramp_s  is None else ramp_s
TAKEOFF_WAIT_S = spool_s + ramp_s + TAKEOFF_SETTLE_S
print(f"  Takeoff ramp: COM_SPOOLUP_TIME={spool_s:.1f} s + MPC_TKO_RAMP_T={ramp_s:.1f} s "
      f"→ wait {TAKEOFF_WAIT_S:.1f} s after arming before climbing")

# ─────────────────────────────────────────────────────────────
# Step 2 – Stream setpoints → OFFBOARD → Arm
# 解鎖前 setpoint = 目前地面位置與航向（高度誤差 0），而不是目標高度
# ─────────────────────────────────────────────────────────────
print("\n[Step 2] Streaming ground-position setpoints (5 s)...")
set_pva(ground, yaw=ground_yaw)
time.sleep(5.0)

print("  Requesting OFFBOARD mode...")
set_mode_offboard()
if not wait_for_mode(6):
    time.sleep(2.5)
    set_mode_offboard()
print("  OFFBOARD confirmed." if wait_for_mode(6) else "  WARNING: OFFBOARD not confirmed.")

print("  Arming...")
time.sleep(1.0)
arm()
if not wait_until(lambda: _hb['armed'], 5.0):
    arm(force=True)
    if not wait_until(lambda: _hb['armed'], 5.0):
        print("  ERROR: Failed to arm（原因見上方 [PX4] / [ARM] 訊息）.")
        shutdown(1)
print("  Armed.")

try:
    # ─────────────────────────────────────────────────────────
    # Step 3 – 平滑起飛：等 PX4 起飛緩升 → 最小 jerk 爬升 → 等高度穩定
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 3a] Holding ground position for {TAKEOFF_WAIT_S:.1f} s (PX4 takeoff ramp)...")
    t_w = time.time()
    while time.time() - t_w < TAKEOFF_WAIT_S:
        check_flight()
        time.sleep(0.05)

    z0   = _ned['z']
    dz   = -TARGET_ALT_M - z0
    T_cl = max(CLIMB_MIN_S, MIN_JERK_PEAK * abs(dz) / CLIMB_PEAK_MPS)

    def _climb(t):
        s, sd, sdd = min_jerk(t, T_cl)
        return ((ground[0], ground[1], z0 + s * dz), (0.0, 0.0, sd * dz),
                (0.0, 0.0, sdd * dz), ground_yaw)
    print(f"\n[Step 3b] Smooth climb {-z0:.2f} m → {TARGET_ALT_M:.2f} m")
    run_trajectory(_climb, T_cl, "climb")
    hover_p = (ground[0], ground[1], -TARGET_ALT_M)
    set_pva(hover_p, yaw=ground_yaw)

    print(f"\n[Step 3c] Waiting for altitude (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")
    stable_since = None
    t_ph = last_log = time.time()
    while True:
        check_flight()
        now = time.time()
        alt_err = TARGET_ALT_M + _ned['z']
        stable_since = (stable_since or now) if abs(alt_err) < ALT_TOL else None
        if now - last_log > 1.0:
            print(f"  alt={-_ned['z']:.2f} m (err={-alt_err:+.2f})")
            last_log = now
        if stable_since and now - stable_since >= HOVER_STABLE_TIME:
            print(f"  Stable at {-_ned['z']:.2f} m.")
            break
        if now - t_ph > ALT_WAIT_TIMEOUT:
            print(f"  Timeout – alt={-_ned['z']:.2f} m, continuing.")
            break
        time.sleep(0.05)

    print(f"\n[Step 3d] Hover hold for {HOVER_PHASE_DUR:.0f} s...")
    t_w = time.time()
    while time.time() - t_w < HOVER_PHASE_DUR:
        check_flight()
        time.sleep(0.05)
    # 路徑中心 = 懸停 setpoint（不是當下估測位置，避免把懸停誤差帶進路徑）
    cn, ce = hover_p[0], hover_p[1]
    print(f"  Path centre: NED ({cn:.2f}, {ce:.2f}) m")

    yaw    = ground_yaw                     # 全程固定航向（全向平移）
    frame  = letter_frame(cn, ce, yaw)
    centre = (cn, ce, -TARGET_ALT_M)
    p_start = to_ned(frame, PIECES[0].start())

    # ─────────────────────────────────────────────────────────
    # Step 4 – 中心 → 第一筆起點
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 4] Approach start point N {p_start[0]:.2f} E {p_start[1]:.2f} alt {-p_start[2]:.2f}")
    T_app = line_duration(centre, p_start, APPROACH_PEAK_MPS, min_dur=3.0)
    run_trajectory(line_segment(centre, p_start, T_app, yaw), T_app, "centre → start")
    turn_and_hold(p_start, yaw, yaw, CORNER_DWELL_S, "start dwell")

    # ─────────────────────────────────────────────────────────
    # Step 5 – 寫字
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 5] Writing {TEXT!r} ({len(PIECES)} pieces, ≈ {_TOTAL_DUR:.0f} s)")
    p_end = p_start
    for i, pc in enumerate(PIECES):
        p_end = run_trajectory(piece_fn(frame, pc, yaw), pc.T,
                               f"{i + 1}/{len(PIECES)} {pc.label} ({pc.kind}, {pc.L:.2f} m @ {pc.V:.2f} m/s)")[0]
        turn_and_hold(p_end, yaw, yaw, CORNER_DWELL_S, "dwell")
    print(f"\n  {TEXT!r} complete.")

    # ─────────────────────────────────────────────────────────
    # Step 6 – 回到中心保持
    # ─────────────────────────────────────────────────────────
    print("\n[Step 6] Returning to centre...")
    T_ret = line_duration(p_end, centre, APPROACH_PEAK_MPS, min_dur=3.0)
    run_trajectory(line_segment(p_end, centre, T_ret, yaw), T_ret, "end → centre")
    turn_and_hold(centre, yaw, yaw, 5.0, "centre hold")

except Aborted as e:
    print(f"\n  ✗ Aborted: {e} → 不再切換模式，交給飛手 / failsafe。")
    shutdown(1)
except KeyboardInterrupt:
    print("\n  Ctrl-C → AUTO.LAND")

# ─────────────────────────────────────────────────────────────
# Land
# ─────────────────────────────────────────────────────────────
print("\n[Land] Landing (AUTO.LAND)...")
set_mode_land()
# 確認已離開 OFFBOARD 才停止 setpoint，否則會先觸發 offboard lost → Position mode
if not wait_for_mode(4, 5.0):
    print("  WARNING: AUTO.LAND not confirmed, retrying...")
    set_mode_land()
    wait_for_mode(4, 5.0)
_sp_active.clear()
print("  讓 AUTO.LAND 自行偵測落地後上鎖，請勿在空中切 RC 上鎖開關。")
time.sleep(15)
_running.clear()
print("Done.")
