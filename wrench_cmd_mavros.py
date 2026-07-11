#!/usr/bin/env python3
"""
Wrench Command via MAVROS (ROS 1 Noetic)
=========================================
透過 MAVROS 發送期望 wrench，讓 pfa 控制鏈與控制分配器計算後，
讀回實際馬達/伺服輸出，驗證期望 vs 實際。

■ 架構說明
  本腳本對應 TVMD 的完整控制鏈：

  DESIRED_WRENCH [Fx, Fy, Fz, Tx, Ty, Tz]
         │
         ├─ 力  [Fx, Fy, Fz]
         │    → PositionTarget (position hold + acceleration feedforward)
         │    → pfa_pos_control → thrust_body[3] (3D 歸一化力向量)
         │
         └─ 力矩 [Tx, Ty, Tz]
              → 姿態誤差（AttitudeTarget 設定期望 roll/pitch/yaw）
              → pfa_att_control → torque_setpoint（歸一化力矩）

  ── 控制分配器 ──
  vehicle_thrust_setpoint + vehicle_torque_setpoint
         → ActuatorEffectivenessVTOL_TVMD
         → 各 agent 馬達油門 + servo 傾斜角
         → SERVO_OUTPUT_RAW (PWM)

  ── 實際輸出觀測 ──
  /mavros/actuator_outputs → 實際 PWM（馬達 0-3：位置 0-3，伺服 0-7：位置 4-11）

■ MAVROS 介面限制
  PX4 OFFBOARD 僅能同時使用一種 setpoint 類型。
  本腳本使用 PositionTarget（含 acc feedforward）控制力，
  力矩透過 PFA_DES_ROLL/PITCH/YAW_BIAS 參數注入（非 AttitudeTarget）。

  如需純力矩控制（無水平力）：使用 /mavros/actuator_control（actuator_controls group 0）
  直接注入 [roll_norm, pitch_norm, yaw_norm, thrust_norm]，跳過 pfa_att_control。

■ 座標系（NWU：North-West-Up）
  +Fx = 前進力（+North）  +Tx = 右滾力矩
  +Fy = 左側力（+West）   +Ty = 仰頭力矩
  +Fz = 向上力（+Up）     +Tz = 左偏力矩
  懸停：Fz ≈ m*g = 13.7 N

■ PWM → 推力換算
  motor_thrust (N) = c_l * (PWM - PWM_min) / (PWM_max - PWM_min)
  servo_angle (deg) = SERVO_MAX * (PWM - PWM_mid) / (PWM_max - PWM_mid)

執行：
  python3 wrench_cmd_mavros.py
"""

import math
import threading
import rospy

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg  import PoseStamped
from sensor_msgs.msg    import Imu
from mavros_msgs.msg    import State, PositionTarget, ParamValue, RCOut
from mavros_msgs.srv    import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration ← 使用者修改這裡
# ─────────────────────────────────────────────────────────────

# 期望 wrench [Fx, Fy, Fz, Tx, Ty, Tz] (N, N·m)，機體 NWU 座標
# 範例：向前 1N + 向左 1N + 懸停力，無力矩
DESIRED_WRENCH = [1.0, 1.0, 14.0, 0.0, 0.0, 0.0]

VEHICLE_MASS_KG  = 1.4      # kg
TARGET_ALT_M     = 1.0      # 懸停高度 (m)
TRACK_DURATION_S = 15.0     # 追蹤持續時間 (s)
STOP_HOLD_S      = 3.0      # 結束後位置保持時間 (s)
CTRL_HZ          = 20       # 控制頻率

# 韌體參數（與 13300_generic_vtol_tvmd 對齊）
PFA_MAX_THR  = 24.0    # N    thrust 歸一化分母
PFA_MAX_TOR  = 15.0    # N·m  torque 歸一化分母
C_L          = 6.125   # N/throttle（馬達推力係數）
SERVO_MAX_DEG = 30.0   # 伺服最大傾斜角 (deg)

# PWM 範圍（用於 actuator_outputs 反推推力與角度）
PWM_MOTOR_MIN = 1000   # µs
PWM_MOTOR_MAX = 2000   # µs
PWM_SERVO_MIN = 1000   # µs
PWM_SERVO_MAX = 2000   # µs
PWM_SERVO_MID = 1500   # µs

# TVMD 馬達/伺服通道映射（對應 SERVO_OUTPUT_RAW，1-indexed）
# 依據 13300_generic_vtol_tvmd：motors ch1-4, servos ch5-12
MOTOR_CHANNELS  = [1, 2, 3, 4]    # Agent 0-3 馬達
SERVO_X_CHANNELS = [5, 7, 9, 11]  # Agent 0-3 X 傾斜伺服
SERVO_Y_CHANNELS = [6, 8, 10, 12] # Agent 0-3 Y 傾斜伺服
AGENT_LABELS = ['Front-Left', 'Front-Right', 'Rear-Left', 'Rear-Right']

G = 9.81

# ─────────────────────────────────────────────────────────────
# PositionTarget type_mask
# ─────────────────────────────────────────────────────────────
# 位置保持 + 加速度前饋 + 偏航（忽略速度, bit9=加速度模式, 忽略偏航率）
MASK_POS_ACC = (8 + 16 + 32   # ignore vx, vy, vz
               + 512           # type = acceleration (not force)
               + 2048)         # ignore yaw_rate
# = 2616

MASK_POS_HOLD = (8 + 16 + 32 + 64 + 128 + 256 + 2048)  # = 2552

# ─────────────────────────────────────────────────────────────
# Subscribers
# ─────────────────────────────────────────────────────────────
_state     = State()
_pose      = PoseStamped()
_imu       = Imu()
_rc_out    = RCOut()          # SERVO_OUTPUT_RAW → 實際 PWM 輸出

def _cb_state(msg):  global _state;  _state  = msg
def _cb_pose(msg):   global _pose;   _pose   = msg
def _cb_imu(msg):    global _imu;    _imu    = msg
def _cb_rc_out(msg): global _rc_out; _rc_out = msg

def get_yaw_rad():
    o = _imu.orientation
    _, _, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return y

def get_rpy_deg():
    o = _imu.orientation
    r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return math.degrees(r), math.degrees(p), math.degrees(y)

def get_alt():
    return _pose.pose.position.z

def get_imu_force_nwu():
    """
    從 IMU 估計實際 body NWU 力向量（N）。
    IMU body frame 為 FLU（MAVROS 慣例），量測包含重力反作用。
    NWU = FLU（x=前, y=左, z=上 都相同）。
    靜止懸停時：Fz_est ≈ m*g。
    """
    a = _imu.linear_acceleration
    return (VEHICLE_MASS_KG * a.x,
            VEHICLE_MASS_KG * a.y,
            VEHICLE_MASS_KG * a.z)

# ─────────────────────────────────────────────────────────────
# PWM → 推力 / 角度 解算
# ─────────────────────────────────────────────────────────────
def pwm_to_motor_thrust(pwm):
    """PWM → 馬達推力（N），線性映射。"""
    throttle = max(0.0, (pwm - PWM_MOTOR_MIN) / (PWM_MOTOR_MAX - PWM_MOTOR_MIN))
    return C_L * throttle

def pwm_to_servo_angle(pwm):
    """PWM → 伺服傾斜角（deg），以中點 1500 為 0。"""
    return SERVO_MAX_DEG * (pwm - PWM_SERVO_MID) / (PWM_SERVO_MAX - PWM_SERVO_MID)

def get_actuator_state():
    """
    從 /mavros/rc/out（SERVO_OUTPUT_RAW）讀取各 agent 的
    馬達推力（N）與 X/Y 傾斜角（deg）。
    """
    ch = _rc_out.channels    # 0-indexed
    if len(ch) < 12:
        return None

    agents = []
    for i in range(4):
        m_pwm = ch[MOTOR_CHANNELS[i] - 1]
        sx_pwm = ch[SERVO_X_CHANNELS[i] - 1]
        sy_pwm = ch[SERVO_Y_CHANNELS[i] - 1]

        F      = pwm_to_motor_thrust(m_pwm)
        eta_x  = pwm_to_servo_angle(sx_pwm)
        eta_y  = pwm_to_servo_angle(sy_pwm)
        agents.append({
            'label': AGENT_LABELS[i],
            'F_N'  : F,
            'eta_x': eta_x,
            'eta_y': eta_y,
            'motor_pwm': m_pwm,
            'sx_pwm': sx_pwm,
            'sy_pwm': sy_pwm,
        })
    return agents

# ─────────────────────────────────────────────────────────────
# PositionTarget 工廠
# ─────────────────────────────────────────────────────────────
def make_pos_hold(x, y, z, yaw):
    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1
    msg.type_mask        = MASK_POS_HOLD
    msg.position.x = x;  msg.position.y = y;  msg.position.z = z
    msg.yaw = yaw
    return msg

def make_wrench_force_sp(hold_x, hold_y, hold_z, Fx_nwu, Fy_nwu, yaw_hold):
    """
    位置保持 + 水平力前饋 setpoint。
    Fx_nwu, Fy_nwu：NWU 機體系水平力（N）→ 轉換為 ENU 加速度前饋。
    Fz（垂直力）由位置控制器自動維持高度。

    NWU → ENU 速度方向：
      ENU.x（東）= NWU_fwd * sin(yaw) + NWU_left * (-cos(yaw))... 依據當前 yaw
    更簡單做法：先把 NWU body force 轉到 ENU world frame：
      ENU.x = Fx_nwu * cos(yaw_ENU) - Fy_nwu * sin(yaw_ENU)   [East]
      ENU.y = Fx_nwu * sin(yaw_ENU) + Fy_nwu * cos(yaw_ENU)   [North]
    其中 yaw_ENU 為 MAVROS 慣例（0=East, CCW positive）。
    """
    yaw = get_yaw_rad()
    ax_enu = (Fx_nwu * math.cos(yaw) - Fy_nwu * math.sin(yaw)) / VEHICLE_MASS_KG
    ay_enu = (Fx_nwu * math.sin(yaw) + Fy_nwu * math.cos(yaw)) / VEHICLE_MASS_KG

    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1
    msg.type_mask        = MASK_POS_ACC
    msg.position.x = hold_x
    msg.position.y = hold_y
    msg.position.z = hold_z
    msg.yaw = yaw_hold
    msg.acceleration_or_force.x = ax_enu
    msg.acceleration_or_force.y = ay_enu
    msg.acceleration_or_force.z = 0.0
    return msg

# ─────────────────────────────────────────────────────────────
# Timer 背景發布
# ─────────────────────────────────────────────────────────────
_sp_lock = threading.Lock()
_sp_msg  = [None]

def set_sp(msg):
    with _sp_lock:
        _sp_msg[0] = msg

def _timer_cb(_event):
    with _sp_lock:
        msg = _sp_msg[0]
    if msg is not None:
        msg.header.stamp = rospy.Time.now()
        sp_pub.publish(msg)

# ─────────────────────────────────────────────────────────────
# 服務輔助
# ─────────────────────────────────────────────────────────────
def param_set(name, value, retries=5):
    try:
        rospy.wait_for_service('/mavros/param/set', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        for _ in range(retries):
            if svc(req).success: return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"param_set({name}): {e}")
    return False

def set_mode(mode_str, retries=5):
    try:
        rospy.wait_for_service('/mavros/set_mode', timeout=5)
        svc = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        for _ in range(retries):
            if svc(SetModeRequest(custom_mode=mode_str)).mode_sent: return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"set_mode({mode_str}): {e}")
    return False

def arm(armed=True, retries=5):
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        for _ in range(retries):
            if svc(CommandBoolRequest(value=armed)).success: return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"arm({armed}): {e}")
    return False

def wait_for_alt(target_z, tol=0.12, stable_s=2.0, timeout_s=30.0):
    stable_since = None
    t0 = rospy.Time.now()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout_s:
            rospy.logwarn("wait_for_alt: timeout"); return False
        if abs(get_alt() - target_z) < tol:
            if stable_since is None: stable_since = rospy.Time.now()
            elif (rospy.Time.now() - stable_since).to_sec() >= stable_s: return True
        else:
            stable_since = None
        r.sleep()
    return False

# ─────────────────────────────────────────────────────────────
# 期望分配預算（離線計算，供對照）
# ─────────────────────────────────────────────────────────────
def _hat(v):
    return [[ 0,-v[2], v[1]],[ v[2],0,-v[0]],[-v[1],v[0],0]]

def _mat_mul(A, B):
    rA,cA = len(A),len(A[0]); rB,cB = len(B),len(B[0])
    C = [[0.0]*cB for _ in range(rA)]
    for i in range(rA):
        for j in range(cB):
            for k in range(cA):
                C[i][j] += A[i][k]*B[k][j]
    return C

def _gauss_inv(M):
    n = len(M)
    aug = [r[:]+[1.0 if i==j else 0.0 for j in range(n)] for i,r in enumerate(M)]
    for col in range(n):
        piv = max(range(col,n), key=lambda r: abs(aug[r][col]))
        aug[col],aug[piv] = aug[piv],aug[col]
        sc = aug[col][col]
        aug[col] = [v/sc for v in aug[col]]
        for row in range(n):
            if row!=col:
                f=aug[row][col]
                aug[row]=[aug[row][k]-f*aug[col][k] for k in range(2*n)]
    return [r[n:] for r in aug]

def _mv(M,v):
    return [sum(M[i][j]*v[j] for j in range(len(v))) for i in range(len(M))]

def expected_allocation(Fx, Fy, Fz, Tx, Ty, Tz):
    """
    用有效性矩陣偽逆計算理論期望分配。
    位置與 az 參數與 13300_generic_vtol_tvmd 一致。
    """
    AGENTS_GEO = [
        ( 0.16,  0.16, 0, 0),  # Front-Left
        ( 0.16, -0.16, 0, 0),  # Front-Right
        (-0.16,  0.16, 0, 0),  # Rear-Left
        (-0.16, -0.16, 0, 0),  # Rear-Right
    ]
    n = len(AGENTS_GEO)
    B = [[0.0]*(3*n) for _ in range(6)]
    for i,(px,py,pz,az) in enumerate(AGENTS_GEO):
        p=[px,py,pz]
        phat = _hat(p)
        Rz_i = [[1,0,0],[0,1,0],[0,0,1]]  # az=0 → identity
        phatRz = _mat_mul(phat,Rz_i)
        for r in range(3):
            for c in range(3):
                B[r  ][3*i+c] = phatRz[r][c]
                B[r+3][3*i+c] = Rz_i[r][c]
    Bt = [[B[r][c] for r in range(6)] for c in range(3*n)]
    BBt = _mat_mul(B, Bt)
    BBt_inv = _gauss_inv(BBt)
    Bpinv = _mat_mul(Bt, BBt_inv)
    W = [Tx,Ty,Tz,Fx,Fy,Fz]
    f_vec = _mv(Bpinv, W)
    results = []
    for i in range(n):
        fx,fy,fz = f_vec[3*i:3*i+3]
        F = math.sqrt(fx**2+fy**2+fz**2)
        etax = math.degrees(math.atan2(fx,fz)) if abs(fz)>1e-9 else math.copysign(90,fx)
        etay = math.degrees(math.atan2(fy,fz)) if abs(fz)>1e-9 else math.copysign(90,fy)
        u_prop = max(0.0,min(1.0,F/C_L))
        results.append({'F':F,'eta_x':etax,'eta_y':etay,'u_prop':u_prop})
    return results

# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main():
    global sp_pub

    Fx, Fy, Fz, Tx, Ty, Tz = DESIRED_WRENCH
    rospy.init_node('wrench_cmd_mavros')
    rate = rospy.Rate(CTRL_HZ)

    # ── Subscribers ──
    rospy.Subscriber('/mavros/state',               State,       _cb_state)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, _cb_pose)
    rospy.Subscriber('/mavros/imu/data',            Imu,         _cb_imu)
    rospy.Subscriber('/mavros/rc/out',              RCOut,       _cb_rc_out)

    # ── Publisher + Timer ──
    sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    rospy.Timer(rospy.Duration(1.0/CTRL_HZ), _timer_cb)
    rospy.sleep(1.0)

    # ── 輸出期望分配（離線預算） ──
    exp = expected_allocation(Fx, Fy, Fz, Tx, Ty, Tz)
    print("=" * 72)
    print("  Wrench Command Test via MAVROS")
    print("=" * 72)
    print(f"\n期望 wrench（NWU）：Fx={Fx:+.2f} Fy={Fy:+.2f} Fz={Fz:+.2f} N  "
          f"Tx={Tx:+.2f} Ty={Ty:+.2f} Tz={Tz:+.2f} N·m")
    print(f"\n理論期望分配（離線偽逆計算）：")
    print(f"  {'Agent':<14} {'F_motor':>8} {'η_x':>7} {'η_y':>7} {'u_prop':>8}")
    for i, r in enumerate(exp):
        warn = " ⚠" if (abs(r['eta_x'])>SERVO_MAX_DEG or abs(r['eta_y'])>SERVO_MAX_DEG) else ""
        print(f"  {AGENT_LABELS[i]:<14} {r['F']:8.3f} N {r['eta_x']:+7.2f}° {r['eta_y']:+7.2f}° {r['u_prop']:8.4f}{warn}")
    print(f"\n控制介面：PositionTarget + acc_ff（力） | PFA_DES_ROLL/PITCH=0（力矩=0）")
    if any(abs(t)>0.1 for t in [Tx,Ty,Tz]):
        print(f"[WARN] 非零力矩 [{Tx:.2f},{Ty:.2f},{Tz:.2f}] N·m 無法透過 PositionTarget 直接注入，")
        print(f"       將忽略力矩命令。如需力矩，請改用 /mavros/actuator_control (group 0)。")

    # ── Step 1：預熱發布 ──
    print("\n[1] 預熱 setpoint...")
    x0 = _pose.pose.position.x
    y0 = _pose.pose.position.y
    yaw0 = get_yaw_rad()
    set_sp(make_pos_hold(x0, y0, TARGET_ALT_M, yaw0))
    rospy.sleep(2.0)

    # ── Step 2：OFFBOARD + 解鎖 ──
    print("[2] OFFBOARD + ARM...")
    if not set_mode('OFFBOARD'): rospy.logerr("OFFBOARD 失敗"); return
    rospy.sleep(0.3)
    if not arm(True): rospy.logerr("ARM 失敗"); return

    # ── Step 3：爬升 ──
    print(f"[3] 爬升至 {TARGET_ALT_M:.1f} m...")
    wait_for_alt(TARGET_ALT_M, stable_s=2.0, timeout_s=30.0)

    hold_x = _pose.pose.position.x
    hold_y = _pose.pose.position.y
    yaw_hold = get_yaw_rad()
    print(f"    懸停鎖定：x={hold_x:.2f} y={hold_y:.2f} yaw={math.degrees(yaw_hold):.1f}°")
    param_set('PFA_DES_ROLL',  0.0)
    param_set('PFA_DES_PITCH', 0.0)
    rospy.sleep(1.0)

    # ── Step 4：追蹤 wrench 力分量 ──
    print(f"\n[4] 追蹤 wrench（{TRACK_DURATION_S:.0f} s）...")
    print(f"\n{'Time':>5} | {'期望 Fx':>7} {'Fy':>7} {'Fz':>7}"
          f" | {'IMU Fx':>7} {'Fy':>7} {'Fz':>7}"
          f" | {'Roll':>6} {'Pitch':>6} {'Alt':>5}")
    print("-" * 75)

    # 收集實際輸出（log）
    actuator_log = []
    t_track = rospy.Time.now()
    log_t   = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - t_track).to_sec()
        if elapsed >= TRACK_DURATION_S: break

        sp = make_wrench_force_sp(hold_x, hold_y, TARGET_ALT_M, Fx, Fy, yaw_hold)
        set_sp(sp)

        if (rospy.Time.now() - log_t).to_sec() >= 0.5:
            log_t = rospy.Time.now()
            ifx, ify, ifz = get_imu_force_nwu()
            roll_d, pitch_d, _ = get_rpy_deg()
            print(f"{elapsed:5.1f} | {Fx:7.2f} {Fy:7.2f} {Fz:7.2f}"
                  f" | {ifx:7.2f} {ify:7.2f} {ifz:7.2f}"
                  f" | {roll_d:6.1f} {pitch_d:6.1f} {get_alt():5.2f}")
            act = get_actuator_state()
            if act:
                actuator_log.append({'t': elapsed, 'agents': act})

        rate.sleep()

    # ── Step 5：停止 ──
    print("\n[5] 移除力前饋，位置保持...")
    set_sp(make_pos_hold(hold_x, hold_y, TARGET_ALT_M, yaw_hold))
    rospy.sleep(STOP_HOLD_S)

    # ── Step 6：降落 ──
    print("[6] AUTO.LAND...")
    set_mode('AUTO.LAND')
    rospy.sleep(5.0)

    # ── 結果比對 ──
    print("\n" + "="*72)
    print("  理論期望 vs 實際輸出（末次 actuator_outputs 讀值）")
    print("="*72)
    if actuator_log:
        last = actuator_log[-1]['agents']
        print(f"\n{'Agent':<14} │ {'期望 F':>8} {'期望η_x':>8} {'期望η_y':>8} "
              f"│ {'實際 F':>8} {'實際η_x':>8} {'實際η_y':>8} │ {'馬達PWM':>8}")
        print("-" * 80)
        for i, (e, a) in enumerate(zip(exp, last)):
            print(f"{AGENT_LABELS[i]:<14} │ {e['F']:8.3f} N {e['eta_x']:+8.2f}° {e['eta_y']:+8.2f}° "
                  f"│ {a['F_N']:8.3f} N {a['eta_x']:+8.2f}° {a['eta_y']:+8.2f}° │ {a['motor_pwm']:8d}")
    else:
        print("  [WARN] 未收到 /mavros/rc/out 資料")
        print("  確認：mavlink stream 需啟用 SERVO_OUTPUT_RAW (stream_rate > 0)")
        print("  或執行：rosrun mavros mavsys tune -r -b")

    print(f"\n說明：")
    print(f"  理論值：離線偽逆計算（假設無飽和，線性映射）")
    print(f"  實際值：PX4 韌體 pfa_att_control → control_allocator → PWM 輸出")
    print(f"  PWM→推力：T = {C_L}×(PWM-{PWM_MOTOR_MIN})/({PWM_MOTOR_MAX}-{PWM_MOTOR_MIN}) N")
    print(f"  PWM→角度：η = {SERVO_MAX_DEG}°×(PWM-{PWM_SERVO_MID})/({PWM_SERVO_MAX}-{PWM_SERVO_MID})")
    print("="*72)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
