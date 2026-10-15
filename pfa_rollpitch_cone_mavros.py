#!/usr/bin/env python3
"""
PFA Roll + Pitch Cone (Coning) Tracking (MAVROS / ROS 1)
=========================================================
pfa_rollpitch_sine_mavros.py 的 90° 相位版本：原地懸停，roll 與 pitch 以相差 90°
的正弦波擺動（預設 ±15°），機身傾斜方向繞圈（錐形擺動），位置維持不動。

■ 姿態指令怎麼送進 PX4
  pfa_pos_control 的期望 roll / pitch 只來自參數 PFA_DES_ROLL / PFA_DES_PITCH
  （yaw 來自 trajectory_setpoint），所以正弦波是以 PARAM_HZ 的頻率連續
  PARAM_SET 這兩個參數；位置 setpoint 由 rospy.Timer 以 CTRL_HZ 在背景持續發布，
  兩者互不阻塞。
    θ_roll(t)  = ROLL_AMP_DEG  · e(t) · sin(ωt)
    θ_pitch(t) = PITCH_AMP_DEG · e(t) · sin(ωt + PITCH_PHASE_DEG)
    ω = 2π / SINE_PERIOD_S
  e(t) 為振幅包絡：前 ENV_RAMP_S 以最小 jerk 曲線 0 → 1，中間 NUM_CYCLES 個
  完整週期 e = 1，最後 ENV_RAMP_S 再 1 → 0。避免一開始角速度就跳到 A·ω。
  本版固定 PITCH_PHASE_DEG = ±90° → 錐形擺動（coning）：
    θ_roll = A·sin(ωt)、θ_pitch = ±A·cos(ωt)，一軸最大時另一軸為 0，
    機頂傾斜方向以週期 SINE_PERIOD_S 繞圈，合成傾角大致維持 A，
    機身頂部畫出一個圓錐；yaw 不變（機頭方向固定）。
    CONE_CCW = True  → 俯視逆時針（機頂依序傾向 後 → 右 → 前 → 左）
    CONE_CCW = False → 俯視順時針（後 → 左 → 前 → 右）
  與同相版（PITCH_PHASE_DEG = 0）相比，合成傾角沒有 √2 倍的尖峰，但每一刻都維持
  約 A 的傾角，舵機持續處於大偏轉；結束時會額外統計實際合成傾角。

■ PARAM_SET 的限制
  /mavros/param/set 是同步服務（等 PARAM_VALUE 回覆），經數傳時一次約 0.1–0.3 s，
  兩個參數一輪可能就要 0.2–0.6 s。指令值以實際時間計算，所以送得慢只會讓
  正弦波取樣變粗、不會讓時間軸漂移；結束時會印出實際更新率。
  每次改參數 PX4 會觸發 parameter_update 並自動存檔，請不要把 PARAM_HZ 調得太高。

■ 韌體每秒只讀一次參數（重要）
  pfa_pos_control 的 _parameter_update_sub 是 SubscriptionInterval 1 s，PFA_DES_ROLL/PITCH
  每秒最多生效一次，所以 PARAM_HZ 預設 = 1：送更快不會更平滑，只會多觸發參數存檔
  （log 會出現 "parameter export to /fs/microsd/params failed"）。1 Hz 階梯的每步大小
  約 A·ω·1 s，預設 ±15°、週期 20 s 時約 4.7°。

■ 位置優先保護（GUARD_*）
  pfa_pos_control 逐軸限幅、沒有「先保垂直推力」：傾角接近舵機極限時，位置一漂，
  Kp·e 要求的水平力會把機身 z 推力擠到 0 而失高度（exp_pitch_20：pitch 25° 墜落）。
  腳本每拍檢查實測傾角、舵機角度（pymavlink 版）、水平與高度誤差，超標就把指令乘上
  g（g ← 0.7·g）讓出姿態、先保位置；條件都有餘裕時 g 再慢慢回到 1。

■ 姿態偏差補償（BIAS_*）
  指令 0° 懸停時實測姿態有穩態偏差（例如 roll ≈ −5°、pitch ≈ +5°：重心 / 電纜力矩，
  姿態迴路沒有積分）。擺動前先以 0° 懸停 BIAS_MEASURE_S 秒量測平均偏差 b，之後送出
  指令 = 參考角 − b，讓「實測」姿態以真正的 0° 為中心擺動（例如 pitch 偏差 +5°、
  振幅 15° 時指令為 −20°…+10°，實測為 −15°…+15°）。補償只移動指令，干擾力矩仍在，
  兩側可用的舵機角度不對稱，所以位置優先保護仍以實測傾角 / 舵機角度判斷。

■ 安全
  - 期間若離開 OFFBOARD 或被上鎖，立即中止並把 PFA_DES_ROLL/PITCH 設回 0
    （這兩個參數在其他飛行模式下也會被 pfa_pos_control 使用），之後不再切換
    模式，交給飛手 / failsafe 處理
  - 任何例外 / Ctrl-C 也會先把兩個參數設回 0 再離開
  - 振幅上限 MAX_AMP_DEG；roll、pitch 各軸需要的機身側向推力約
    T_hover·sin(θ)，必須小於 pfa_pos_control 的 _thrust_xy_max = 0.3

■ 起飛
  與 hover_test_smooth_mavros.py 相同的平滑起飛：解鎖前 setpoint = 地面位置與航向，
  等 PX4 起飛緩升（COM_SPOOLUP_TIME + MPC_TKO_RAMP_T）結束後以最小 jerk 曲線爬升。
  OFFBOARD 下必須 PFA_TKF_BYP = 1（啟動時會檢查）。

■ 姿態量測的正負號
  /mavros/imu/data 的姿態是 ENU / FLU（ROS REP-103）：roll 與 PX4 同號，
  pitch 與 PX4 相反（FLU 的 +pitch 是機頭朝下）。本腳本印出與統計時一律換成
  PX4 慣例：pitch 正值 = 機頭朝上，roll 正值 = 右翼朝下。

執行方式：
  python3 pfa_rollpitch_cone_mavros.py
"""

import sys
import math
import threading

import rospy
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ParamValue, PositionTarget
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
    ParamGet,    ParamGetRequest,
    ParamPull,   ParamPullRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 1.0    # 懸停高度 (m)；光流 SENS_FLOW_MINHGT = 0.8 m，建議 ≥ 1.0

ROLL_AMP_DEG      = 15.0   # roll 振幅 (deg)，正值 = 右翼朝下
PITCH_AMP_DEG     = 15.0   # pitch 振幅 (deg)，正值 = 機頭朝上
SINE_PERIOD_S     = 20.0   # 正弦週期 (s)；峰值角速度 = 2π·A/T（15°, 20 s → 4.7°/s）
CONE_CCW          = True   # True: 俯視逆時針（後→右→前→左）; False: 順時針
PITCH_PHASE_DEG   = 90.0 if CONE_CCW else -90.0   # pitch 相對 roll 的相位 (deg)
NUM_CYCLES        = 2      # 滿振幅的完整週期數
ENV_RAMP_S        = SINE_PERIOD_S   # 振幅包絡淡入 / 淡出時間 (s)
MAX_AMP_DEG       = 18.0   # 振幅硬上限 (deg)；20° 起舵機就接近飽和（exp_pitch_20）

PARAM_HZ          = 1.0    # PFA_DES_ROLL/PITCH 更新頻率 (Hz)；韌體每秒只讀一次參數
PARAM_DEADBAND_DEG = 0.2   # 與上次送出值差異小於此值就不送

# 位置優先保護：接近舵機 / 推力極限時縮小姿態指令，先保住位置與高度、不硬追姿態
# 指令 = g × A·e(t)·sin(…)，g ∈ [0, 1]；任一條件超標時 g ← g × GUARD_BACKOFF，
# 全部條件都有餘裕時 g 每秒回升 GUARD_RECOVER_PER_S
GUARD_TILT_DEG      = 20.0   # 實測 |roll| 或 |pitch| 超過就縮小指令
GUARD_SERVO_DEG     = 20.0   # 舵機偏轉超過就縮小指令（舵機極限 ±30°，留 10° 給位置修正）
GUARD_XY_ERR_M      = 0.10   # 水平位置誤差上限 (m)
GUARD_ALT_ERR_M     = 0.15   # 高度誤差上限 (m)
GUARD_BACKOFF       = 0.7    # 超標時每拍乘上的係數
GUARD_RECOVER_PER_S = 0.1    # 有餘裕時 g 每秒回升量

# 姿態偏差補償：指令 0° 懸停時實測姿態有穩態偏差（重心 / 電纜力矩、姿態迴路沒有積分，
# 例如 roll ≈ −5°、pitch ≈ +5°）。水平懸停的最後 BIAS_MEASURE_S 秒量測平均偏差，
# 之後送出的指令 = 參考角 − 偏差，讓「實測」姿態追參考角（以真正的 0° 為基準）。
# 注意：補償只移動指令，干擾力矩仍在，舵機用掉的角度不變（兩側可用角度不對稱）。
BIAS_COMP         = True    # False = 不補償（指令 = 參考角）
BIAS_MEASURE_S    = 5.0     # 量測時間 (s)：PFA_DES_ROLL/PITCH = 0° 懸停，取實測平均
BIAS_MAX_DEG      = 8.0     # 偏差上限 (deg)：超過視為異常，補償量限制在 ±BIAS_MAX_DEG
BIAS_SETTLE_S     = 3.0     # 套用補償後等待姿態穩定、再驗證殘差的時間 (s)

HOVER_BEFORE_S    = 5.0    # 開始擺動前的懸停時間 (s)
HOVER_AFTER_S     = 5.0    # 擺動結束、姿態歸零後的懸停時間 (s)
ALT_TOL           = 0.25   # 高度穩定容許誤差 (m)
HOVER_STABLE_TIME = 2.0    # 高度在容許範圍內需持續的時間 (s)
ALT_WAIT_TIMEOUT  = 15.0   # 等待高度穩定的上限 (s)

# 平滑起飛
CLIMB_PEAK_MPS    = 0.2    # 爬升最小 jerk 曲線的峰值速度 (m/s)
CLIMB_MIN_S       = 3.0    # 爬升最短時間 (s)
TAKEOFF_SETTLE_S  = 0.5    # PX4 起飛緩升結束後再多等的時間 (s)
MIN_JERK_PEAK     = 1.875  # 最小 jerk 曲線：峰值速度 = 1.875 × 距離 / 時間

CTRL_HZ           = 20     # setpoint 發布頻率（經 SiK 數傳時建議降到 10）
LOG_PERIOD_S      = 0.5    # 追蹤狀態印出間隔 (s)

# PositionTarget.type_mask：位置、速度、加速度、yaw 都使用；只忽略 yaw_rate
TYPE_MASK_PVA_YAW = PositionTarget.IGNORE_YAW_RATE

ZERO3 = (0.0, 0.0, 0.0)

_OMEGA       = 2.0 * math.pi / SINE_PERIOD_S
_PHASE_RAD   = math.radians(PITCH_PHASE_DEG)
_FULL_S      = NUM_CYCLES * SINE_PERIOD_S
_TOTAL_S     = 2.0 * ENV_RAMP_S + _FULL_S

if max(abs(ROLL_AMP_DEG), abs(PITCH_AMP_DEG)) > MAX_AMP_DEG:
    sys.exit(f"ROLL/PITCH_AMP_DEG 超過 MAX_AMP_DEG = {MAX_AMP_DEG}°")

# ─────────────────────────────────────────────────────────────
# State
# ─────────────────────────────────────────────────────────────
_state = State()
_pose  = PoseStamped()
_imu   = Imu()
_pose_received = threading.Event()

def _cb_state(msg): global _state; _state = msg
def _cb_imu(msg):   global _imu;   _imu   = msg
def _cb_pose(msg):
    global _pose
    _pose = msg
    _pose_received.set()

def get_alt(): return _pose.pose.position.z

def get_xyz():
    p = _pose.pose.position
    return p.x, p.y, p.z

def get_yaw():
    o = _pose.pose.orientation
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

def get_roll_pitch_px4_deg():
    """IMU 姿態（ENU/FLU）換成 PX4 慣例：roll 同號，pitch 反號（正值 = 機頭朝上）。"""
    o = _imu.orientation
    roll, pitch, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(roll), -math.degrees(pitch)

# ─────────────────────────────────────────────────────────────
# Shared setpoint (Timer 讀取；主執行緒寫入)
# ─────────────────────────────────────────────────────────────
_sp      = {'p': ZERO3, 'v': ZERO3, 'a': ZERO3, 'yaw': 0.0}
_sp_lock = threading.Lock()

def update_sp(p, v=ZERO3, a=ZERO3, yaw=None):
    """更新 ENU setpoint（位置 / 速度 / 加速度 / yaw），由 Timer 在背景發布。"""
    with _sp_lock:
        _sp['p'], _sp['v'], _sp['a'] = tuple(p), tuple(v), tuple(a)
        if yaw is not None:
            _sp['yaw'] = yaw

def make_target(p, v, a, yaw):
    """ENU PositionTarget；三軸皆為有限值（pfa_pos_control 遇 NaN 會整個向量清 0）。"""
    sp = PositionTarget()
    sp.header.stamp     = rospy.Time.now()
    sp.header.frame_id  = 'map'
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    sp.type_mask        = TYPE_MASK_PVA_YAW
    sp.position.x, sp.position.y, sp.position.z = p
    sp.velocity.x, sp.velocity.y, sp.velocity.z = v
    sp.acceleration_or_force.x, sp.acceleration_or_force.y, sp.acceleration_or_force.z = a
    sp.yaw      = yaw
    sp.yaw_rate = 0.0
    return sp

def _timer_publish(_event):
    with _sp_lock:
        p, v, a, yaw = _sp['p'], _sp['v'], _sp['a'], _sp['yaw']
    sp_pub.publish(make_target(p, v, a, yaw))

def min_jerk(t, T):
    """最小 jerk 曲線 s(t)∈[0,1]：回傳 (s, ṡ, s̈)，起訖速度與加速度皆為 0。"""
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return 1.0, 0.0, 0.0
    u = t / T
    return (u**3 * (10.0 - 15.0 * u + 6.0 * u * u),
            30.0 * u * u * (1.0 - u) ** 2 / T,
            60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T))

# ─────────────────────────────────────────────────────────────
# Sine profile
# ─────────────────────────────────────────────────────────────
def envelope(t):
    """振幅包絡：淡入（最小 jerk）→ 1 → 淡出（最小 jerk）。"""
    if t < ENV_RAMP_S:
        return min_jerk(t, ENV_RAMP_S)[0]
    if t < ENV_RAMP_S + _FULL_S:
        return 1.0
    return 1.0 - min_jerk(t - ENV_RAMP_S - _FULL_S, ENV_RAMP_S)[0]

def guard_update(g, roll_m, pitch_m, xy_e, alt_e, servo_deg, dt):
    """位置優先保護：回傳 (新的 g, 超標原因)。"""
    reasons = []
    if max(abs(roll_m), abs(pitch_m)) > GUARD_TILT_DEG:
        reasons.append(f"tilt {max(abs(roll_m), abs(pitch_m)):.0f}°")
    if servo_deg is not None and servo_deg > GUARD_SERVO_DEG:
        reasons.append(f"servo {servo_deg:.0f}°")
    if xy_e > GUARD_XY_ERR_M:
        reasons.append(f"xy {xy_e:.2f} m")
    if abs(alt_e) > GUARD_ALT_ERR_M:
        reasons.append(f"alt {alt_e:+.2f} m")
    if reasons:
        return g * GUARD_BACKOFF, reasons
    roomy = (max(abs(roll_m), abs(pitch_m)) < GUARD_TILT_DEG - 3.0
             and (servo_deg is None or servo_deg < GUARD_SERVO_DEG - 3.0)
             and xy_e < 0.5 * GUARD_XY_ERR_M and abs(alt_e) < 0.5 * GUARD_ALT_ERR_M)
    return (min(1.0, g + GUARD_RECOVER_PER_S * dt) if roomy else g), []

def attitude_cmd_deg(t):
    """t 秒時的 (roll, pitch) 指令 (deg，PX4 慣例)。"""
    e = envelope(t)
    return (ROLL_AMP_DEG  * e * math.sin(_OMEGA * t),
            PITCH_AMP_DEG * e * math.sin(_OMEGA * t + _PHASE_RAD))

# ─────────────────────────────────────────────────────────────
# Service helpers
# ─────────────────────────────────────────────────────────────
def param_pull(force=True, timeout=15.0):
    """Sync MAVROS parameter cache from FCU (prevents stale-cache param_set failures)."""
    try:
        rospy.wait_for_service('/mavros/param/pull', timeout=timeout)
        svc = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
        res = svc(ParamPullRequest(force_pull=force))
        if res.success:
            rospy.loginfo(f"  param_pull: synced {res.param_received} parameters from FCU")
        else:
            rospy.logwarn("  param_pull: reported failure")
        return res.success
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"  param_pull failed: {e}")
        return False

_param_set_svc = None

def param_set(name, value):
    global _param_set_svc
    try:
        if _param_set_svc is None:
            rospy.wait_for_service('/mavros/param/set', timeout=5)
            _param_set_svc = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        return _param_set_svc(req).success
    except Exception as e:
        rospy.logwarn(f"param_set({name}): {e}")
        return False

def param_get(name):
    """讀取 FCU 參數（float 或 int），失敗回傳 None。"""
    try:
        rospy.wait_for_service('/mavros/param/get', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        res = svc(ParamGetRequest(param_id=name))
        if not res.success:
            return None
        return res.value.real if res.value.integer == 0 else float(res.value.integer)
    except Exception as e:
        rospy.logwarn(f"param_get({name}): {e}")
        return None

def set_mode(mode):
    try:
        rospy.wait_for_service('/mavros/set_mode', timeout=5)
        svc = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        return svc(SetModeRequest(custom_mode=mode)).mode_sent
    except Exception as e:
        rospy.logwarn(f"set_mode({mode}): {e}")
        return False

def arm():
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        return svc(CommandBoolRequest(value=True)).success
    except Exception as e:
        rospy.logwarn(f"arming: {e}")
        return False

def sample_attitude(duration: float):
    """取樣 duration 秒的實測 roll / pitch (deg)。
    回傳 (roll 平均, pitch 平均, roll 標準差, pitch 標準差)；setpoint 由 Timer 持續發送。"""
    rs, ps = [], []
    r = rospy.Rate(20)
    t_end = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < t_end:
        roll_m, pitch_m = get_roll_pitch_px4_deg()
        rs.append(roll_m)
        ps.append(pitch_m)
        r.sleep()
    if not rs:
        return 0.0, 0.0, 0.0, 0.0
    mr, mp = sum(rs) / len(rs), sum(ps) / len(ps)
    sr = math.sqrt(sum((v - mr) ** 2 for v in rs) / len(rs))
    sp = math.sqrt(sum((v - mp) ** 2 for v in ps) / len(ps))
    return mr, mp, sr, sp


def measure_and_apply_bias():
    """量測姿態偏差並把補償寫入 PFA_DES_ROLL/PITCH；回傳 (bias_roll, bias_pitch) deg。"""
    rospy.loginfo(f"\n[Step 3b] Measuring attitude bias for {BIAS_MEASURE_S:.0f} s "
          f"(PFA_DES_ROLL/PITCH = 0°)...")
    mr, mp, sr, sp = sample_attitude(BIAS_MEASURE_S)
    rospy.loginfo(f"  measured roll {mr:+.1f}° (σ {sr:.1f}°)   pitch {mp:+.1f}° (σ {sp:.1f}°)")
    br = max(-BIAS_MAX_DEG, min(BIAS_MAX_DEG, mr))
    bp = max(-BIAS_MAX_DEG, min(BIAS_MAX_DEG, mp))
    if (br, bp) != (mr, mp):
        rospy.logwarn(f"  bias exceeds ±{BIAS_MAX_DEG:.0f}° → clipped to "
              f"roll {br:+.1f}°, pitch {bp:+.1f}°（請檢查重心 / 電纜）")
    rospy.loginfo(f"\n[Step 3c] Bias compensation: PFA_DES_ROLL = {-br:+.1f}°, "
          f"PFA_DES_PITCH = {-bp:+.1f}°")
    for name, val in (('PFA_DES_ROLL', -br), ('PFA_DES_PITCH', -bp)):
        ok = param_set(name, round(val, 2))
        rospy.loginfo(f"  {name} = {val:+.1f}° ... {'OK' if ok else 'FAILED'}")
    rospy.sleep(max(0.0, BIAS_SETTLE_S - 2.0))
    cr, cp, _, _ = sample_attitude(2.0)
    rospy.loginfo(f"  after compensation: roll {cr:+.1f}°  pitch {cp:+.1f}°  (目標 0°)")
    return br, bp


def zero_attitude_params():
    """PFA_DES_ROLL/PITCH 設回 0（重試數次）。"""
    for name in ('PFA_DES_ROLL', 'PFA_DES_PITCH'):
        for _ in range(5):
            if param_set(name, 0.0):
                break
            rospy.sleep(0.2)
        else:
            rospy.logerr(f"  ✗ {name} 無法設回 0，請立即在 QGC 手動設定！")

def flight_ok():
    return _state.mode == 'OFFBOARD' and _state.armed

# ─────────────────────────────────────────────────────────────
# ROS init
# ─────────────────────────────────────────────────────────────
rospy.init_node('pfa_rollpitch_cone', anonymous=False)
rate = rospy.Rate(CTRL_HZ)

rospy.Subscriber('/mavros/state',               State,       _cb_state)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)
sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

# ─────────────────────────────────────────────────────────────
# 等待 FCU 連線與位置估測
# ─────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not _state.connected:
    rate.sleep()
rospy.loginfo(f"  Connected!  mode={_state.mode}  armed={_state.armed}")

rospy.loginfo("Waiting for local position...")
while not rospy.is_shutdown() and not _pose_received.is_set():
    rate.sleep()

# 解鎖前 setpoint = 目前地面位置與航向（高度誤差 0），而不是 TARGET_ALT_M
ground     = get_xyz()
ground_yaw = get_yaw()
update_sp(ground, yaw=ground_yaw)
rospy.loginfo(f"  Ground position: ENU ({ground[0]:.2f}, {ground[1]:.2f}, {ground[2]:.2f}) m  "
              f"yaw={math.degrees(ground_yaw):.1f}°")
rospy.loginfo(f"  Plan (cone, {'CCW' if CONE_CCW else 'CW'}): alt={TARGET_ALT_M} m  roll ±{ROLL_AMP_DEG}°  pitch ±{PITCH_AMP_DEG}°  "
              f"period={SINE_PERIOD_S} s  phase={PITCH_PHASE_DEG}°  "
              f"{ENV_RAMP_S:.0f} s fade-in + {NUM_CYCLES} cycles + {ENV_RAMP_S:.0f} s fade-out "
              f"= {_TOTAL_S:.0f} s  (PARAM_SET @ {PARAM_HZ} Hz)")
rospy.loginfo(f"  Peak rate: roll {math.degrees(_OMEGA) * math.radians(ROLL_AMP_DEG):.1f}°/s  "
              f"pitch {math.degrees(_OMEGA) * math.radians(PITCH_AMP_DEG):.1f}°/s")

# Timer 在背景執行緒以 CTRL_HZ 持續發布 setpoint
# 確保 arm() / set_mode() / param_set() 阻塞期間 PX4 不會 timeout
_sp_timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_publish)

# ─────────────────────────────────────────────────────────────
# Step 0: PFA 姿態參數歸零、讀取起飛緩升與推力參數
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 0] Syncing parameter cache, PFA_DES_ROLL/PITCH = 0°")
param_pull()
rospy.loginfo("  PFA_DES_ROLL  ... " + ("OK" if param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
rospy.loginfo("  PFA_DES_PITCH ... " + ("OK" if param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

tkf_byp = param_get('PFA_TKF_BYP')
spool_s = param_get('COM_SPOOLUP_TIME')
ramp_s  = param_get('MPC_TKO_RAMP_T')
mass    = param_get('PFA_VEH_MASS')
f_max   = param_get('PFA_MAX_THR')
if tkf_byp is not None and int(round(tkf_byp)) == 0:
    rospy.logwarn("  WARNING: PFA_TKF_BYP = 0 → OFFBOARD 下收不到 want_takeoff，"
                  "推力上限會卡在 0.1，請改回 1。")
spool_s = 1.0 if spool_s is None else spool_s
ramp_s  = 3.0 if ramp_s  is None else ramp_s
TAKEOFF_WAIT_S = spool_s + ramp_s + TAKEOFF_SETTLE_S
rospy.loginfo(f"  Takeoff ramp: COM_SPOOLUP_TIME={spool_s:.1f} s + MPC_TKO_RAMP_T={ramp_s:.1f} s "
              f"→ wait {TAKEOFF_WAIT_S:.1f} s after arming before climbing")

# 各軸機身側向推力 ≈ T_hover·sin(θ)，需小於 _thrust_xy_max = 0.3
if mass and f_max:
    t_hover = mass * 9.81 / f_max
    worst   = max(abs(ROLL_AMP_DEG), abs(PITCH_AMP_DEG))
    need    = t_hover * math.sin(math.radians(worst))
    rospy.loginfo(f"  Hover thrust (PFA_VEH_MASS/PFA_MAX_THR) ≈ {t_hover:.2f} → "
                  f"lateral thrust at {worst:.0f}° ≈ {need:.2f} (limit 0.30)")
    if need > 0.25:
        rospy.logwarn("  WARNING: 側向推力接近 _thrust_xy_max 0.3，位置可能保持不住；"
                      "若實際懸停推力比參數大（PFA_MAX_THR 偏高），餘裕會更小。")

# ─────────────────────────────────────────────────────────────
# Step 1: 串流 setpoint → OFFBOARD → 解鎖
# ─────────────────────────────────────────────────────────────
rospy.loginfo("\n[Step 1] Streaming ground-position setpoints for 5 s...")
rospy.sleep(5.0)   # Timer 持續發布，主執行緒只需等待

rospy.loginfo("  Requesting OFFBOARD mode...")
set_mode('OFFBOARD')
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'OFFBOARD':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  OFFBOARD not confirmed, continuing...")
        break
    rate.sleep()
rospy.loginfo(f"  Mode: {_state.mode}")

rospy.loginfo("  Arming...")
arm()   # 阻塞服務呼叫；Timer 在背景持續發布 setpoint，PX4 不會 timeout
t0 = rospy.Time.now()
while not rospy.is_shutdown() and not _state.armed:
    if (rospy.Time.now() - t0).to_sec() > 10.0:
        rospy.logerr("  ✗ Failed to arm. Check QGC for pre-arm errors.")
        _sp_timer.shutdown()
        sys.exit(1)
    rate.sleep()
rospy.loginfo("  ✓ Armed!")

# ─────────────────────────────────────────────────────────────
# Step 2: 平滑起飛（等 PX4 起飛緩升 → 最小 jerk 爬升）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 2a] Holding ground position for {TAKEOFF_WAIT_S:.1f} s (PX4 takeoff ramp)...")
rospy.sleep(TAKEOFF_WAIT_S)

z0   = get_alt()
dz   = TARGET_ALT_M - z0
T_cl = max(CLIMB_MIN_S, MIN_JERK_PEAK * abs(dz) / CLIMB_PEAK_MPS)
rospy.loginfo(f"\n[Step 2b] Smooth climb {z0:.2f} m → {TARGET_ALT_M:.2f} m in {T_cl:.1f} s "
              f"(feedforward: p + v + a)")
t0       = rospy.Time.now()
last_log = t0
while not rospy.is_shutdown():
    now = rospy.Time.now()
    t   = (now - t0).to_sec()
    s, sd, sdd = min_jerk(t, T_cl)
    update_sp((ground[0], ground[1], z0 + s * dz), (0.0, 0.0, sd * dz), (0.0, 0.0, sdd * dz))
    if (now - last_log).to_sec() >= LOG_PERIOD_S:
        rospy.loginfo(f"    {t:4.1f}s  alt_cmd={z0 + s * dz:.2f} m  alt={get_alt():.2f} m")
        last_log = now
    if t >= T_cl:
        break
    rate.sleep()
hover_p = (ground[0], ground[1], TARGET_ALT_M)
update_sp(hover_p)   # 爬升結束：保持目標高度，前饋歸零

rospy.loginfo(f"\n[Step 2c] Waiting for altitude (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s, "
              f"timeout {ALT_WAIT_TIMEOUT:.0f} s)...")
stable_since = None
t0 = rospy.Time.now()
while not rospy.is_shutdown():
    now = rospy.Time.now()
    stable_since = (stable_since or now) if abs(TARGET_ALT_M - get_alt()) < ALT_TOL else None
    if stable_since and (now - stable_since).to_sec() >= HOVER_STABLE_TIME:
        rospy.loginfo(f"  Stable at {get_alt():.2f} m.")
        break
    if (now - t0).to_sec() > ALT_WAIT_TIMEOUT:
        rospy.logwarn(f"  Timeout – alt={get_alt():.2f} m, continuing.")
        break
    rate.sleep()

rospy.loginfo(f"\n[Step 3] Level hover for {HOVER_BEFORE_S:.0f} s...")
rospy.sleep(HOVER_BEFORE_S)
bias_r, bias_p = measure_and_apply_bias() if BIAS_COMP else (0.0, 0.0)

# ─────────────────────────────────────────────────────────────
# Step 4: roll + pitch 正弦追蹤（PARAM_SET @ PARAM_HZ）
# ─────────────────────────────────────────────────────────────
rospy.loginfo(f"\n[Step 4] Roll/pitch sine tracking for {_TOTAL_S:.0f} s")
rospy.loginfo(f"  {'t':>5}  {'env':>4}  {'g':>4}  {'roll_ref':>8} {'roll':>6}  {'pitch_ref':>9} {'pitch':>6}  "
              f"{'alt':>5}  {'xy_err':>6}")

sent       = {'PFA_DES_ROLL': round(-bias_r, 2), 'PFA_DES_PITCH': round(-bias_p, 2)}   # FCU 目前的值（已確認）
n_updates  = 0
n_fail     = 0
err_r, err_p, err_xy = [], [], []
tilt_cmd, tilt_meas = [], []   # 合成傾角 (deg)
aborted    = False
param_dt   = 1.0 / PARAM_HZ
g_safe     = 1.0         # 位置優先保護係數
g_min      = 1.0
n_guard    = 0

try:
    t0       = rospy.Time.now()
    last_log = t0
    next_tick = 0.0
    while not rospy.is_shutdown():
        t = (rospy.Time.now() - t0).to_sec()
        if t >= _TOTAL_S:
            break
        if not flight_ok():
            rospy.logerr(f"  ✗ Left OFFBOARD / disarmed (mode={_state.mode}, armed={_state.armed}) "
                         f"→ aborting sine")
            aborted = True
            break

        # 位置優先保護：先看實測狀態，超標就縮小指令（MAVROS 版沒有舵機回授，以實測傾角近似）
        roll_m, pitch_m = get_roll_pitch_px4_deg()
        xn, yn, zn      = get_xyz()
        xy_e            = math.hypot(hover_p[0] - xn, hover_p[1] - yn)
        g_safe, why = guard_update(g_safe, roll_m, pitch_m, xy_e, TARGET_ALT_M - zn, None, param_dt)
        if why:
            n_guard += 1
            rospy.logwarn(f"  [guard] {', '.join(why)} → g = {g_safe:.2f}（讓出姿態、位置優先）")
        g_min = min(g_min, g_safe)

        # 以實際時間計算指令；PARAM_SET 慢時只會取樣變粗，不會漂移
        # roll_m / pitch_m 量測時生效中的參考角 = 這一拍更新前 FCU 的參數值 + 偏差
        ref_r, ref_p = sent['PFA_DES_ROLL'] + bias_r, sent['PFA_DES_PITCH'] + bias_p

        # 參考角 = g·A·sin(…)；送出 參考角 − 偏差
        roll_c, pitch_c = (g_safe * v for v in attitude_cmd_deg(t))
        for name, val in (('PFA_DES_ROLL', roll_c - bias_r), ('PFA_DES_PITCH', pitch_c - bias_p)):
            if abs(val - sent[name]) >= PARAM_DEADBAND_DEG:
                if param_set(name, round(val, 2)):
                    sent[name] = round(val, 2)
                    n_updates += 1
                else:
                    n_fail += 1

        # 追蹤誤差：實測與量測當下生效的參考角比較
        if ENV_RAMP_S <= t < ENV_RAMP_S + _FULL_S:
            err_r.append(ref_r - roll_m)
            err_p.append(ref_p - pitch_m)
            err_xy.append(xy_e)
            tilt_cmd.append(math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(ref_r)) * math.cos(math.radians(ref_p)))))))
            tilt_meas.append(math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(roll_m)) * math.cos(math.radians(pitch_m)))))))

        now = rospy.Time.now()
        if (now - last_log).to_sec() >= LOG_PERIOD_S:
            rospy.loginfo(f"  {t:5.1f}  {envelope(t):4.2f}  {g_safe:4.2f}  {ref_r:+8.1f} {roll_m:+6.1f}  "
                          f"{ref_p:+9.1f} {pitch_m:+6.1f}  {zn:5.2f}  {xy_e:6.2f}")
            last_log = now

        next_tick += param_dt
        wait = next_tick - (rospy.Time.now() - t0).to_sec()
        if wait > 0:
            rospy.sleep(wait)
        else:
            next_tick = (rospy.Time.now() - t0).to_sec()   # 落後就不補送，從現在重新排程
    elapsed = (rospy.Time.now() - t0).to_sec()
finally:
    rospy.loginfo("  PFA_DES_ROLL/PITCH → 0°")
    zero_attitude_params()

rospy.loginfo(f"\n  PARAM_SET: {n_updates} updates in {elapsed:.1f} s "
              f"(≈ {n_updates / max(elapsed, 1e-3) / 2:.1f} Hz per axis), {n_fail} failed")
rospy.loginfo(f"  Guard: triggered {n_guard} times, min g = {g_min:.2f}")

def _rms(x): return math.sqrt(sum(v * v for v in x) / len(x)) if x else float('nan')
if err_r:
    rospy.loginfo(f"  Full-amplitude phase: roll err RMS {_rms(err_r):.1f}° (max {max(map(abs, err_r)):.1f}°)  "
                  f"pitch err RMS {_rms(err_p):.1f}° (max {max(map(abs, err_p)):.1f}°)  "
                  f"xy err RMS {_rms(err_xy):.2f} m (max {max(err_xy):.2f} m)")
    rospy.loginfo(f"  Total tilt: cmd {min(tilt_cmd):.1f}–{max(tilt_cmd):.1f}°  "
                  f"measured mean {sum(tilt_meas) / len(tilt_meas):.1f}° "
                  f"(range {min(tilt_meas):.1f}–{max(tilt_meas):.1f}°)")

# ─────────────────────────────────────────────────────────────
# Step 5: 水平懸停 → 降落
# ─────────────────────────────────────────────────────────────
if aborted:
    # 已被 RC / failsafe 接手：不再切模式，避免搶走飛手的控制
    _sp_timer.shutdown()
    rospy.logwarn("  Aborted – 不切換模式，交由目前模式 / 飛手處理。")
    sys.exit(1)

rospy.loginfo(f"\n[Step 5] Level hover for {HOVER_AFTER_S:.0f} s...")
rospy.sleep(HOVER_AFTER_S)

rospy.loginfo("\n[Land] Landing (AUTO.LAND)...")
set_mode('AUTO.LAND')
# 確認已離開 OFFBOARD 才停止 Timer，否則會先觸發 offboard lost → Position mode
t0 = rospy.Time.now()
while not rospy.is_shutdown() and _state.mode != 'AUTO.LAND':
    if (rospy.Time.now() - t0).to_sec() > 5.0:
        rospy.logwarn("  AUTO.LAND not confirmed, retrying...")
        set_mode('AUTO.LAND')
        break
    rate.sleep()
_sp_timer.shutdown()
rospy.loginfo("  讓 AUTO.LAND 自行偵測落地後上鎖，請勿在空中切 RC 上鎖開關。")
rospy.sleep(15.0)
rospy.loginfo("Done.")
