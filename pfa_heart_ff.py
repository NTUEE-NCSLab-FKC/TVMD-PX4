#!/usr/bin/env python3
"""
PFA Heart Trajectory with Feedforward (pymavlink)
==================================================
繞愛心軌跡飛行，roll = pitch = 0°，yaw 可固定或跟著路線轉（YAW_TRACK_PATH），
同時送出位置 + 速度 + 加速度前饋。

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

■ 愛心曲線（經典參數式，t ∈ [0, 2π)）
    x = 16·sin³t
    y = 13·cos t − 5·cos 2t − 2·cos 3t − cos 4t
  縮放成寬 HEART_WIDTH_M。上方凹處（t = 0）與下方尖端（t = π）都是尖點
  （速度必須為 0），所以一圈分成右半與左半兩段：
    凹處 → 右半邊 → 尖端（停 CORNER_DWELL_S）→ 左半邊 → 凹處（停）
  CLOCKWISE = False 時反向（先左半邊）。

■ 機頭方向 YAW_TRACK_PATH
  False：yaw 全程固定為起飛航向（全向平移，最乾淨）
  True ：機頭沿路線切線方向（只支援 PLANE = 'horizontal'）
         - 曲線上 yaw 角速度 = v·|κ|，另外限制 v ≤ YAW_RATE_MAX_DPS / |κ|
         - 凹處、尖端切線方向不連續：停下後以最小 jerk 曲線轉向（峰值 YAW_PEAK_RATE_DPS），
           再停 YAW_SETTLE_S 才出發；起點先轉到起始切線方向，結束後在中心轉回起飛航向

■ 速度曲線（與 pfa_hershey_ff.py 相同）
  v ≤ V_MAX_MPS，且 v ≤ √(A_LAT_MAX / |κ|)（彎越急越慢），
  前後各掃一次套上切向加速度上限 A_TAN_MAX；每段起停速度為 0。
  前饋：v = T·v(s)，a = T·v·dv/ds + κ·v²（含向心加速度）。

■ 書寫平面 PLANE
  'horizontal'：TARGET_ALT_M 高度的水平面（俯視；愛心的「上」= 起飛時機頭方向）
  'vertical'  ：垂直面（站在飛機後方看；尖端高度 = ALT_BOTTOM_M）

■ 預覽（不連線、不起飛）
  python3 pfa_heart_ff.py --preview
  → 印出各段長度與時間，輸出 heart_path.svg（顏色 = 速度）與 heart_path.csv

■ 連線（MAVProxy 分流時）
  mavproxy.py --master=/dev/ttyUSB0 --baud 57600 \\
              --out udpout:127.0.0.1:14550 --out udpin:0.0.0.0:14551
  預設連 udpout:127.0.0.1:14551（MAVROS 不能同時使用 14551）；也可用參數指定：
    python3 pfa_heart_ff.py udp:127.0.0.1:14550    （QGC 不能同時佔 14550）
"""

import sys
import time
import math
import bisect
import threading
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
PLANE          = 'horizontal'   # 'horizontal'：水平面；'vertical'：垂直面
TARGET_ALT_M   = 1.0     # 懸停高度 (m)；horizontal 時也是飛行高度
ALT_BOTTOM_M   = 0.6     # vertical 時愛心尖端的高度 (m)
HEART_WIDTH_M  = 1.6     # 愛心寬度 (m)；高度約 0.92 × 寬度
NUM_LAPS       = 2       # 圈數
CLOCKWISE      = True    # True: 凹處 → 右半邊 → 尖端 → 左半邊；False: 反向
V_MAX_MPS      = 0.3     # 最高速度 (m/s)
A_LAT_MAX      = 0.3     # 向心加速度上限 (m/s²)：決定彎道速度
A_TAN_MAX      = 0.3     # 切向加速度上限 (m/s²)
CORNER_DWELL_S = 1.0     # 尖點（凹處、尖端）停留時間 (s)
YAW_TRACK_PATH = False   # True: 機頭沿切線方向（只支援 horizontal）；False: 固定起飛航向
YAW_RATE_MAX_DPS = 60.0  # YAW_TRACK_PATH 時曲線上的 yaw 角速度上限 (°/s) → v ≤ ω/|κ|
YAW_SETTLE_S   = 1.0     # YAW_TRACK_PATH 時尖點轉向完成後再停留的時間 (s)
APPROACH_PEAK_MPS = 0.3  # 中心 ↔ 起點的最小 jerk 峰值速度 (m/s)
SMOOTH_M       = 0.01    # 取樣平滑尺度 (m)（解析曲線本身已平滑）
TRANSIT_SPEED_MPS = 0.2  # （未使用；SampledPiece 需要）
RAMP_S         = 1.0     # （未使用；SampledPiece 需要）

# MAVLink 連線：可用第一個參數覆寫，例如
#   python3 pfa_heart_ff.py udp:127.0.0.1:14550        （監聽 MAVProxy 的 udpout；QGC 不能同時佔 14550）
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
# Sampled path + curvature-limited speed profile（同 pfa_hershey_ff.py）
# ─────────────────────────────────────────────────────────────
DS = 0.01   # 取樣間距 (m)


def _resample(pts, ds):
    out, carry = [pts[0]], 0.0
    for a, b in zip(pts, pts[1:]):
        L = math.dist(a, b)
        if L < 1e-9:
            continue
        s = ds - carry
        while s <= L:
            out.append((a[0] + (b[0] - a[0]) * s / L, a[1] + (b[1] - a[1]) * s / L))
            s += ds
        carry = L - (s - ds)
    if math.dist(out[-1], pts[-1]) > 1e-6:
        out.append(pts[-1])
    return out


def _split_cusps(pts):
    """重新取樣後，以 ±CUSP_WIN_M 的前後方向夾角找尖點（局部最大且 > CUSP_DEG）並切開。

    用一段距離的方向而不是相鄰頂點，才不會把 Hershey 整數座標的小折角當成尖點。
    """
    r = _resample(pts, DS)
    n, k = len(r), max(1, int(round(CUSP_WIN_M / DS)))
    if n < 3:
        return [r] if n == 2 else []
    ang = [0.0] * n
    for i in range(k, n - k):
        a, b, c = r[i - k], r[i], r[i + k]
        d1 = (b[0] - a[0], b[1] - a[1]); d2 = (c[0] - b[0], c[1] - b[1])
        ang[i] = abs(math.degrees(math.atan2(d1[0] * d2[1] - d1[1] * d2[0], d1[0] * d2[0] + d1[1] * d2[1])))
    cuts = [i for i in range(k, n - k)
            if ang[i] > CUSP_DEG and ang[i] >= max(ang[max(0, i - k):i + k + 1])]
    segs, prev = [], 0
    for i in cuts:
        if i - prev >= 2:
            segs.append(r[prev:i + 1])
            prev = i
    segs.append(r[prev:])
    out = []                                   # 太短的段（< 3 cm）併入前一段，不留缺口
    for sg in segs:
        if out and (len(sg) - 1) * DS < 0.03:
            out[-1] = out[-1] + sg[1:]
        elif out and (len(out[-1]) - 1) * DS < 0.03:
            out[-1] = out[-1] + sg[1:]
        else:
            out.append(sg)
    return [sg for sg in out if len(sg) >= 2]


def _smooth(pts, sigma):
    """高斯平滑（端點固定、靠近端點時窗口縮小）。"""
    n, h = len(pts), max(1, int(round(2.5 * sigma / DS)))
    out = []
    for i in range(n):
        k = min(h, i, n - 1 - i)
        if k == 0:
            out.append(pts[i])
            continue
        ws = [math.exp(-0.5 * (j * DS / sigma) ** 2) for j in range(-k, k + 1)]
        W = sum(ws)
        out.append((sum(w * pts[i + j][0] for w, j in zip(ws, range(-k, k + 1))) / W,
                    sum(w * pts[i + j][1] for w, j in zip(ws, range(-k, k + 1))) / W))
    return out


class SampledPiece:
    """取樣路徑（間距約 DS）＋曲率限速的速度曲線，介面同 pfa_ntu_ff.py 的 Piece。"""

    def __init__(self, pts, kind, label):
        self.kind, self.label = kind, label
        if kind == 'transit':
            pts = _resample(pts, DS)
        else:
            pts = _resample(_smooth(_resample(pts, DS), SMOOTH_M), DS)
        self.p = pts
        n = len(pts)
        self.s = [0.0]
        for a, b in zip(pts, pts[1:]):
            self.s.append(self.s[-1] + math.dist(a, b))
        self.L = self.s[-1]
        # 切線、曲率向量（中央差分）
        self.tg, self.kv = [], []
        for i in range(n):
            a, b = pts[max(0, i - 1)], pts[min(n - 1, i + 1)]
            d = math.dist(a, b) or 1e-9
            self.tg.append(((b[0] - a[0]) / d, (b[1] - a[1]) / d))
        for i in range(n):
            a, b = self.tg[max(0, i - 1)], self.tg[min(n - 1, i + 1)]
            ds = (self.s[min(n - 1, i + 1)] - self.s[max(0, i - 1)]) or 1e-9
            self.kv.append(((b[0] - a[0]) / ds, (b[1] - a[1]) / ds))
        # 速度曲線
        if kind == 'transit':
            vmax, alat, atan = TRANSIT_SPEED_MPS, 1e9, 1.5 * TRANSIT_SPEED_MPS / RAMP_S
        else:
            vmax, alat, atan = V_MAX_MPS, A_LAT_MAX, A_TAN_MAX
        v = [min(vmax, math.sqrt(alat / max(math.hypot(*k), 1e-9)), _extra_vlim(k)) for k in self.kv]
        v[0] = v[-1] = 0.0
        for i in range(1, n):
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * atan * (self.s[i] - self.s[i - 1])))
        for i in range(n - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * atan * (self.s[i + 1] - self.s[i])))
        self.v = v
        self.t = [0.0]
        for i in range(1, n):
            vm = 0.5 * (v[i] + v[i - 1])
            self.t.append(self.t[-1] + (self.s[i] - self.s[i - 1]) / max(vm, 1e-3))
        self.T = self.t[-1]
        self.V = max(v)

    def start(self):
        return self.p[0]

    def end(self):
        return self.p[-1]

    def heading_dir(self, t):
        """t 秒時路線的單位切線（2D，幾何切線，與速度大小無關）。"""
        t = min(max(t, 0.0), self.T)
        i = max(0, min(len(self.t) - 2, bisect.bisect_right(self.t, t) - 1))
        w = (t - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9)
        return (self.tg[i][0] + (self.tg[i + 1][0] - self.tg[i][0]) * w,
                self.tg[i][1] + (self.tg[i + 1][1] - self.tg[i][1]) * w)

    def start_dir(self):
        """起點附近 3 cm 的方向（尖點處的幾何切線不穩定）。"""
        a, b = self.p[0], self.p[min(3, len(self.p) - 1)]
        d = math.dist(a, b) or 1e-9
        return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)

    def end_dir(self):
        a, b = self.p[max(0, len(self.p) - 4)], self.p[-1]
        d = math.dist(a, b) or 1e-9
        return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)

    def state2d(self, t):
        t = min(max(t, 0.0), self.T)
        i = max(0, min(len(self.t) - 2, bisect.bisect_right(self.t, t) - 1))
        w = (t - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9)
        lerp = lambda a, b: a + (b - a) * w
        p = (lerp(self.p[i][0], self.p[i + 1][0]), lerp(self.p[i][1], self.p[i + 1][1]))
        tg = (lerp(self.tg[i][0], self.tg[i + 1][0]), lerp(self.tg[i][1], self.tg[i + 1][1]))
        k = (lerp(self.kv[i][0], self.kv[i + 1][0]), lerp(self.kv[i][1], self.kv[i + 1][1]))
        sp = lerp(self.v[i], self.v[i + 1])
        dvds = (self.v[i + 1] - self.v[i]) / max(self.s[i + 1] - self.s[i], 1e-9)
        at = sp * dvds
        return p, (tg[0] * sp, tg[1] * sp), (tg[0] * at + k[0] * sp * sp, tg[1] * at + k[1] * sp * sp)



def heart_halves():
    """回傳 [右半, 左半]（或反向）的取樣點，u 右、v 上（公尺）。"""
    n = 600
    raw = []
    for k in range(n + 1):
        t = 2.0 * math.pi * k / n
        raw.append((16.0 * math.sin(t) ** 3,
                    13.0 * math.cos(t) - 5.0 * math.cos(2 * t) - 2.0 * math.cos(3 * t) - math.cos(4 * t)))
    sc = HEART_WIDTH_M / 32.0
    vmin, vmax = min(p[1] for p in raw), max(p[1] for p in raw)
    vc = (vmax + vmin) / 2.0 if PLANE == 'horizontal' else vmin
    pts = [(x * sc, (y - vc) * sc) for x, y in raw]
    right, left = pts[:n // 2 + 1], pts[n // 2:]
    if CLOCKWISE:
        return [('right half', right), ('left half', left)]
    return [('left half', left[::-1]), ('right half', right[::-1])]


if PLANE not in ('horizontal', 'vertical'):
    sys.exit(f"  ERROR: PLANE 必須是 'horizontal' 或 'vertical'（目前 {PLANE!r}）")
if YAW_TRACK_PATH and PLANE != 'horizontal':
    sys.exit("  ERROR: YAW_TRACK_PATH 只支援 PLANE = 'horizontal'（垂直面的切線含上下方向）")


def _extra_vlim(k):
    """yaw 跟隨路線時，限制 yaw 角速度 v·|κ| ≤ YAW_RATE_MAX_DPS。"""
    if not YAW_TRACK_PATH:
        return float('inf')
    return math.radians(YAW_RATE_MAX_DPS) / max(math.hypot(*k), 1e-9)


_HALVES = [SampledPiece(p, 'draw', name) for name, p in heart_halves()]
PIECES = []
for _lap in range(NUM_LAPS):
    for _h in _HALVES:
        PIECES.append(_h)
_TOTAL_DUR = sum(p.T for p in PIECES) + CORNER_DWELL_S * len(PIECES)
_US = [q[0] for p in _HALVES for q in p.p]
_VS = [q[1] for p in _HALVES for q in p.p]
TEXT = 'heart'


def _summary():
    lap_len = sum(p.L for p in _HALVES)
    lap_T = sum(p.T for p in _HALVES) + 2 * CORNER_DWELL_S
    print(f"  Heart on {PLANE} plane: {max(_US) - min(_US):.2f} × {max(_VS) - min(_VS):.2f} m, "
          f"{lap_len:.2f} m/lap, {lap_T:.1f} s/lap, {NUM_LAPS} laps ≈ {_TOTAL_DUR:.0f} s "
          f"({'CW' if CLOCKWISE else 'CCW'})")
    print(f"    v ≤ {V_MAX_MPS:.2f} m/s, a_lat ≤ {A_LAT_MAX:.2f}, a_tan ≤ {A_TAN_MAX:.2f} m/s²"
          + (f", yaw follows path (≤ {YAW_RATE_MAX_DPS:.0f}°/s on curves)" if YAW_TRACK_PATH else ", yaw fixed"))
    for p in _HALVES:
        kmax = max(math.hypot(*k) for k in p.kv[5:-5])
        print(f"    {p.label:10s} L={p.L:4.2f} m  vmax={p.V:4.2f} m/s  T={p.T:4.1f} s  "
              f"min radius ≈ {1 / kmax:.2f} m")


def _preview():
    rows, t_acc = [], 0.0
    for i, p in enumerate(PIECES):
        n = max(2, int(p.T * 20))
        for k in range(n + 1):
            t = p.T * k / n
            (u, v), (vu, vv), (au, av) = p.state2d(t)
            rows.append((t_acc + t, i, p.kind, u, v, math.hypot(vu, vv), math.hypot(au, av)))
        t_acc += p.T + CORNER_DWELL_S
    with open('heart_path.csv', 'w') as f:
        f.write('t,piece,kind,u_right_m,v_up_m,speed_mps,accel_mps2\n')
        for r in rows:
            f.write(f"{r[0]:.2f},{r[1]},{r[2]},{r[3]:.4f},{r[4]:.4f},{r[5]:.4f},{r[6]:.4f}\n")
    pad, sc = 0.15, 220.0
    u0, v1 = min(_US) - pad, max(_VS) + pad
    w, h = (max(_US) - min(_US) + 2 * pad) * sc, (max(_VS) - min(_VS) + 2 * pad) * sc + 30
    X = lambda u: (u - u0) * sc
    Y = lambda v: (v1 - v) * sc
    col = lambda sp: f"hsl({240 - 24 * round(10 * min(sp / V_MAX_MPS, 1))},80%,40%)"
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" style="background:#fff">']
    for i, p in enumerate(PIECES):
        pts = [r for r in rows if r[1] == i]
        if p.kind == 'transit':
            svg.append('<polyline points="' + ' '.join(f"{X(r[3]):.1f},{Y(r[4]):.1f}" for r in pts)
                       + '" fill="none" stroke="#aaa" stroke-width="2" stroke-dasharray="7 5"/>')
            continue
        run = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            run.append(b)
            if col(b[5]) != col(run[0][5]) or b is pts[-1]:
                svg.append('<polyline points="' + ' '.join(f"{X(r[3]):.1f},{Y(r[4]):.1f}" for r in run)
                           + f'" fill="none" stroke="{col(run[0][5])}" stroke-width="5" stroke-linecap="round"/>')
                run = [b]
    s0 = PIECES[0].start()
    svg.append(f'<circle cx="{X(s0[0]):.1f}" cy="{Y(s0[1]):.1f}" r="7" fill="#2a7"/>')
    svg.append(f'<text x="10" y="{h - 10:.0f}" font-family="sans-serif" font-size="12" fill="#333">'
               f'color = speed (blue 0 → red {V_MAX_MPS:.2f} m/s), green = start</text>')
    svg.append('</svg>')
    with open('heart_path.svg', 'w') as f:
        f.write('\n'.join(svg))
    print("  Wrote heart_path.svg and heart_path.csv")



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


warn_speed(max(V_MAX_MPS, APPROACH_PEAK_MPS), "heart")
if PLANE == 'vertical' and ALT_BOTTOM_M < 0.4:
    print("  WARNING: ALT_BOTTOM_M < 0.4 m，接近地面效應與光流最低高度，建議 ≥ 0.5 m")


def dir_yaw(frame, d2):
    """字母平面的 2D 方向 → NED 航向。"""
    n, e, _ = vec_ned(frame, d2)
    return math.atan2(e, n)


def piece_fn_track(frame, piece):
    """同 piece_fn，但 yaw = 路線切線方向。"""
    def fn(t):
        p, v, a = piece.state2d(t)
        return to_ned(frame, p), vec_ned(frame, v), vec_ned(frame, a), dir_yaw(frame, piece.heading_dir(t))
    return fn

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

    frame   = letter_frame(cn, ce, ground_yaw)
    centre  = (cn, ce, -TARGET_ALT_M)
    p_start = to_ned(frame, PIECES[0].start())

    def piece_yaws(pc):
        if not YAW_TRACK_PATH:
            return ground_yaw, ground_yaw
        return dir_yaw(frame, pc.start_dir()), dir_yaw(frame, pc.end_dir())

    def cusp_turn(p, yaw0, yaw1, title):
        """尖點：停 CORNER_DWELL_S；yaw 跟隨時改為最小 jerk 轉向 + YAW_SETTLE_S。"""
        if YAW_TRACK_PATH and abs(wrap_pi(yaw1 - yaw0)) > 1e-3:
            turn_and_hold(p, yaw0, yaw1, max(CORNER_DWELL_S, yaw_turn_duration(yaw0, yaw1)),
                          f"{title}: yaw {math.degrees(yaw0):.0f}° → {math.degrees(yaw1):.0f}°")
            turn_and_hold(p, yaw1, yaw1, YAW_SETTLE_S, f"{title} settle")
        else:
            turn_and_hold(p, yaw1, yaw1, CORNER_DWELL_S, title)

    # ─────────────────────────────────────────────────────────
    # Step 4 – 中心 → 起點（上方凹處），轉到起始方向
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 4] Approach start point (top dent) N {p_start[0]:.2f} E {p_start[1]:.2f} alt {-p_start[2]:.2f}")
    T_app = line_duration(centre, p_start, APPROACH_PEAK_MPS, min_dur=3.0)
    run_trajectory(line_segment(centre, p_start, T_app, ground_yaw), T_app, "centre → start")
    yaw = ground_yaw
    cusp_turn(p_start, yaw, piece_yaws(PIECES[0])[0], "start (top dent)")

    # ─────────────────────────────────────────────────────────
    # Step 5 – 愛心
    # ─────────────────────────────────────────────────────────
    print(f"\n[Step 5] Heart ({len(PIECES)} pieces, ≈ {_TOTAL_DUR:.0f} s, "
          f"yaw {'follows path' if YAW_TRACK_PATH else 'fixed'})")
    p_end = p_start
    for i, pc in enumerate(PIECES):
        fn = piece_fn_track(frame, pc) if YAW_TRACK_PATH else piece_fn(frame, pc, ground_yaw)
        p_end, _, _, yaw = run_trajectory(fn, pc.T, f"{i + 1}/{len(PIECES)} {pc.label} "
                                                     f"({pc.L:.2f} m, vmax {pc.V:.2f} m/s)")
        nxt = piece_yaws(PIECES[i + 1])[0] if i + 1 < len(PIECES) else yaw
        cusp_turn(p_end, yaw, nxt, "top dent" if pc.label == PIECES[-1].label else "bottom tip")
    print("\n  Heart complete.")

    # ─────────────────────────────────────────────────────────
    # Step 6 – 回到中心，轉回起飛航向並保持
    # ─────────────────────────────────────────────────────────
    print("\n[Step 6] Returning to centre...")
    T_ret = line_duration(p_end, centre, APPROACH_PEAK_MPS, min_dur=3.0)
    run_trajectory(line_segment(p_end, centre, T_ret, yaw), T_ret, "end → centre")
    turn_and_hold(centre, yaw, ground_yaw, max(5.0, yaw_turn_duration(yaw, ground_yaw)),
                  "centre hold, yaw → takeoff heading")

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
