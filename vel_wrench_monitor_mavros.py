#!/usr/bin/env python3
"""
vel_wrench_monitor_mavros.py
============================
透過 MAVROS 發送速度指令，並即時顯示：
  1. 期望 wrench（Newton-Euler 逆動力學，依當前 IMU 姿態計算）
  2. 期望各 agent 推力分配（偽逆）
  3. 實際各 agent 輸出（從 /mavros/rc/out PWM 反推）

■ MAVROS 指令路徑：
  PositionTarget (body FLU, FRAME_BODY_NED=8)
    → trajectory_setpoint → pfa_pos_control → vehicle_attitude_setpoint (thrust_body)
    → pfa_att_control → vehicle_thrust/torque_setpoint
    → control_allocator → actuator PWM

■ 角速度指令說明：
  PositionTarget 無角速度欄位。若需角速度控制，可同時發布
  AttitudeTarget (type_mask=IGNORE_ATTITUDE=128, body_rate 欄位)，
  但 PX4 OFFBOARD 模式下兩者可能衝突——預設腳本僅用 PositionTarget。

■ 期望 wrench = Newton-Euler 逆動力學（機體 NWU / FLU 系）：
  F_body = Rᵀ_ENU · (R_NWU→ENU · (m·v̇_des + m·[0,0,g]))
  τ_body = I·ω̇_des + ω × (I·ω)

■ 執行：
  python3 vel_wrench_monitor_mavros.py
"""

import math
import rospy
from mavros_msgs.msg import PositionTarget, RCOut, State
from sensor_msgs.msg import Imu

# ════════════════════════════════════════════════════════════════
# Configuration ← 修改這裡
# ════════════════════════════════════════════════════════════════

# ── 速度指令（機體 FLU：Forward-Left-Up） ──
VEL_FWD   = 0.5   # m/s  前向（+x）
VEL_LEFT  = 0.0   # m/s  左側（+y）
VEL_UP    = 0.0   # m/s  垂直（+z）

# ── Newton-Euler 期望加速度（世界 NWU 系） ──
# 等速（v_dot=0）→ Fz=m*g（純懸停）
# 前向加速 0.5 m/s² → Fx=m*0.5, Fz=m*g
V_DOT_DES_NWU = [0.5, 0.0, 0.0]   # m/s²  線加速度（NWU：+x=前, +y=左, +z=上）
W_DOT_DES_NWU = [0.0, 0.0, 0.0]   # rad/s² 角加速度（機體 NWU）

# ── 飛行器參數 ──
VEHICLE_MASS = 1.4    # kg
G            = 9.81   # m/s²
I_XX         = 0.015  # kg·m²  Roll 慣性
I_YY         = 0.015  # kg·m²  Pitch 慣性
I_ZZ         = 0.025  # kg·m²  Yaw 慣性

# ── 韌體推力/伺服參數 ──
C_L          = 6.125  # N/throttle  (ActuatorEffectivenessVTOL_TVMD.hpp)
TF0          = 0.0    # N bias
SERVO_MAX_DEG = 30.0  # deg (CA_SV_TL*_MAXA)

# ── TVMD 幾何（H-frame, NWU，對應 13300_generic_vtol_tvmd） ──
AGENTS = [
    ( 0.16,  0.16, 0.0, 0, 'FL'),
    ( 0.16, -0.16, 0.0, 0, 'FR'),
    (-0.16,  0.16, 0.0, 0, 'RL'),
    (-0.16, -0.16, 0.0, 0, 'RR'),
]

# ── PWM 通道（RCOut.channels，0-index） ──
MOTOR_CH   = [0, 1, 2, 3]
SERVO_X_CH = [4, 6, 8, 10]
SERVO_Y_CH = [5, 7, 9, 11]

# ── PositionTarget type_mask（純速度模式） ──
# SET bit = IGNORE；忽略位置(0-2)、加速度(6-8)、偏航(10)、偏航率(11)
MASK_VEL_ONLY = 1+2+4+64+128+256+1024+2048   # = 3527

# ════════════════════════════════════════════════════════════════
# 純 Python 矩陣工具
# ════════════════════════════════════════════════════════════════
def mat_zero(r, c): return [[0.0]*c for _ in range(r)]

def mat_T(M):
    return [[M[i][j] for i in range(len(M))] for j in range(len(M[0]))]

def mat_mul(A, B):
    C = mat_zero(len(A), len(B[0]))
    for i in range(len(A)):
        for j in range(len(B[0])):
            for k in range(len(A[0])):
                C[i][j] += A[i][k] * B[k][j]
    return C

def mv_mul(M, v):
    return [sum(M[i][j]*v[j] for j in range(len(v))) for i in range(len(M))]

def mat_inv(M):
    n = len(M)
    aug = [row[:] + [1.0 if i==j else 0.0 for j in range(n)]
           for i, row in enumerate(M)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        aug[col], aug[pivot] = aug[pivot], aug[col]
        aug[col] = [v / aug[col][col] for v in aug[col]]
        for row in range(n):
            if row != col:
                f = aug[row][col]
                aug[row] = [aug[row][k] - f*aug[col][k] for k in range(2*n)]
    return [row[n:] for row in aug]

def mat_pinv(B):
    Bt = mat_T(B)
    return mat_mul(Bt, mat_inv(mat_mul(B, Bt)))

def cross3(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def quat_to_R(q):
    """quaternion [x,y,z,w] → 3×3 body→world 旋轉矩陣。"""
    x, y, z, w = q
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)  ],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)  ],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ]

def Rz_mat(psi_rad):
    c, s = math.cos(psi_rad), math.sin(psi_rad)
    return [[c,-s,0.0],[s,c,0.0],[0.0,0.0,1.0]]

# NWU → ENU 轉換矩陣（世界系）
# NWU: x=North, y=West, z=Up  →  ENU: x=East, y=North, z=Up
# North→ENU_y, West→-ENU_x → R_NWU_to_ENU = [[0,-1,0],[1,0,0],[0,0,1]]
R_NWU_to_ENU = [[0.0,-1.0,0.0],[1.0,0.0,0.0],[0.0,0.0,1.0]]

# ════════════════════════════════════════════════════════════════
# Newton-Euler 期望 wrench
# ════════════════════════════════════════════════════════════════
def expected_wrench(v_dot_nwu, w_dot_nwu, w_cur_flu, R_body_to_ENU,
                    mass, ixx, iyy, izz, g=9.81):
    """
    給定期望加速度，計算機體 NWU(=FLU) 系的所需 wrench。

    步驟：
      F_world_NWU = m·v̇_des + m·[0,0,g]    (重力補償 + 加速需求)
      F_world_ENU = R_NWU_to_ENU · F_world_NWU
      F_body_FLU  = R_body_to_ENU^T · F_world_ENU   (body FLU ≡ body NWU)
      τ_body      = I·ω̇_des + ω_cur × (I·ω_cur)
    """
    # 世界系（NWU）需求推力
    F_nwu = [mass*v_dot_nwu[i] + (mass*g if i==2 else 0.0) for i in range(3)]
    # 轉到 ENU
    F_enu = mv_mul(R_NWU_to_ENU, F_nwu)
    # 旋轉到機體系（ENU → FLU via R^T）
    F_body = mv_mul(mat_T(R_body_to_ENU), F_enu)
    # 陀螺項
    I_d  = [ixx, iyy, izz]
    Iw   = [I_d[j]*w_cur_flu[j] for j in range(3)]
    gyro = cross3(w_cur_flu, Iw)
    tau  = [I_d[j]*w_dot_nwu[j] + gyro[j] for j in range(3)]
    return F_body, tau

# ════════════════════════════════════════════════════════════════
# Allocation solver（同 wrench_allocation_solver.py）
# ════════════════════════════════════════════════════════════════
def build_B(agents):
    n = len(agents)
    B = mat_zero(6, 3*n)
    for i, (px, py, pz, az_psi, _) in enumerate(agents):
        p   = [px, py, pz]
        R   = Rz_mat(math.pi/2 * az_psi)
        ph  = [[0,-p[2],p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]]
        phR = mat_mul(ph, R)
        for r in range(3):
            for c in range(3):
                B[r][3*i+c]   = phR[r][c]
                B[r+3][3*i+c] = R[r][c]
    return B

def parse_agent(f3):
    fx, fy, fz = f3
    F    = math.sqrt(fx**2+fy**2+fz**2)
    ex   = math.degrees(math.atan2(fx, fz)) if abs(fz) > 1e-9 else math.copysign(90.0, fx)
    ey   = math.degrees(math.atan2(fy, fz)) if abs(fz) > 1e-9 else math.copysign(90.0, fy)
    u    = max(0.0, min(1.0, (F - TF0) / C_L))
    return F, ex, ey, u

# ════════════════════════════════════════════════════════════════
# PWM → 實際 wrench
# ════════════════════════════════════════════════════════════════
def pwm_to_agent_force(pwm_m, pwm_sx, pwm_sy):
    """給定 motor/servoX/servoY 的 PWM，返回局部力向量 [fx,fy,fz]（NWU）。"""
    thr = max(0.0, (pwm_m - 1000) / 1000.0)
    F   = C_L * thr + TF0
    ex  = math.radians(SERVO_MAX_DEG * (pwm_sx - 1500) / 500.0)
    ey  = math.radians(SERVO_MAX_DEG * (pwm_sy - 1500) / 500.0)
    return [F * math.sin(ex),
            F * math.sin(ey),
            F * math.cos(ex) * math.cos(ey)]

def pwm_to_wrench(channels):
    """從 RCOut channels 反推總 wrench [F, τ]（各 3D 向量）。"""
    F_tot = [0.0, 0.0, 0.0]
    T_tot = [0.0, 0.0, 0.0]
    for i, (px, py, pz, _, _) in enumerate(AGENTS):
        f   = pwm_to_agent_force(channels[MOTOR_CH[i]],
                                  channels[SERVO_X_CH[i]],
                                  channels[SERVO_Y_CH[i]])
        tau = cross3([px, py, pz], f)
        for k in range(3):
            F_tot[k] += f[k]
            T_tot[k] += tau[k]
    return F_tot, T_tot

# ════════════════════════════════════════════════════════════════
# ROS 全域狀態
# ════════════════════════════════════════════════════════════════
_imu   = None
_rcout = None
_state = None

def cb_imu(msg):   global _imu;   _imu   = msg
def cb_rc(msg):    global _rcout; _rcout = msg
def cb_state(msg): global _state; _state = msg

# ════════════════════════════════════════════════════════════════
# 即時顯示
# ════════════════════════════════════════════════════════════════
_B     = build_B(AGENTS)
_Bpinv = mat_pinv(_B)

def display():
    print("\033[H\033[J", end='')
    print("═"*70)
    print("  vel_wrench_monitor ─ TVMD 速度指令 + wrench 即時監控")
    print("═"*70)

    armed = _state.armed if _state else False
    mode  = _state.mode  if _state else "?"
    print(f"  狀態：{'ARMED' if armed else 'DISARMED'}  |  Mode: {mode}\n")

    # ── 當前姿態（來自 IMU） ──
    R_b2e  = [[1,0,0],[0,1,0],[0,0,1]]
    w_cur  = [0.0, 0.0, 0.0]
    if _imu:
        q     = _imu.orientation
        R_b2e = quat_to_R([q.x, q.y, q.z, q.w])
        wv    = _imu.angular_velocity
        w_cur = [wv.x, wv.y, wv.z]
        R     = R_b2e
        roll  = math.degrees(math.atan2(R[2][1], R[2][2]))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2][0]))))
        yaw   = math.degrees(math.atan2(R[1][0], R[0][0]))
        print(f"  當前姿態（ENU）：roll={roll:.1f}°  pitch={pitch:.1f}°  yaw={yaw:.1f}°")
        print(f"  當前角速度     ：[{w_cur[0]:.3f}  {w_cur[1]:.3f}  {w_cur[2]:.3f}] rad/s\n")
    else:
        print("  （等待 /mavros/imu/data...）\n")

    # ── 速度指令 ──
    print(f"  速度指令（機體 FLU）：fwd={VEL_FWD:.2f}  left={VEL_LEFT:.2f}  up={VEL_UP:.2f}  m/s")
    print(f"  期望加速度（NWU）  ：{V_DOT_DES_NWU}  m/s²")
    print(f"  ※ 角速度須另用 AttitudeTarget body_rate（見腳本說明）\n")

    # ── 期望 wrench（Newton-Euler） ──
    F_exp, T_exp = expected_wrench(
        V_DOT_DES_NWU, W_DOT_DES_NWU, w_cur, R_b2e,
        VEHICLE_MASS, I_XX, I_YY, I_ZZ, G,
    )
    print(f"  期望 wrench（機體 NWU，Newton-Euler）：")
    print(f"    力  ：Fx={F_exp[0]:+7.3f}  Fy={F_exp[1]:+7.3f}  Fz={F_exp[2]:+7.3f}  N  (懸停參考 Fz≈{VEHICLE_MASS*G:.1f})")
    print(f"    力矩：Tx={T_exp[0]:+7.3f}  Ty={T_exp[1]:+7.3f}  Tz={T_exp[2]:+7.3f}  N·m")

    # ── 期望各 agent 分配 ──
    W_e  = [T_exp[0], T_exp[1], T_exp[2], F_exp[0], F_exp[1], F_exp[2]]
    fv   = mv_mul(_Bpinv, W_e)
    print(f"\n  期望各 agent 分配（偽逆）：")
    print(f"  {'Ag':>4} {'fx':>7} {'fy':>7} {'fz':>7}  {'F(N)':>6}  {'η_x':>6} {'η_y':>6}  {'u_prop':>7}")
    print("  " + "-"*64)
    for i, (*_, lbl) in enumerate(AGENTS):
        f3      = fv[3*i:3*i+3]
        F, ex, ey, u = parse_agent(f3)
        warn = " ✗超限" if (abs(ex) > SERVO_MAX_DEG or abs(ey) > SERVO_MAX_DEG) else ""
        print(f"  {lbl:>4} {f3[0]:+7.3f} {f3[1]:+7.3f} {f3[2]:+7.3f}  "
              f"{F:6.3f}  {ex:+6.2f}° {ey:+6.2f}°  {u:.4f}{warn}")

    # ── 實際 wrench（PWM 反推） ──
    print(f"\n  實際 wrench（/mavros/rc/out PWM 反推）：")
    if _rcout and len(_rcout.channels) >= 12:
        chs = _rcout.channels
        F_act, T_act = pwm_to_wrench(chs)
        print(f"    力  ：Fx={F_act[0]:+7.3f}  Fy={F_act[1]:+7.3f}  Fz={F_act[2]:+7.3f}  N")
        print(f"    力矩：Tx={T_act[0]:+7.3f}  Ty={T_act[1]:+7.3f}  Tz={T_act[2]:+7.3f}  N·m")

        print(f"\n  各 agent 實際 PWM → 力：")
        print(f"  {'Ag':>4} {'Motor':>6} {'Svx':>6} {'Svy':>6}  {'F(N)':>6}  {'η_x':>6} {'η_y':>6}")
        print("  " + "-"*54)
        for i, (*_, lbl) in enumerate(AGENTS):
            pm  = chs[MOTOR_CH[i]]
            psx = chs[SERVO_X_CH[i]]
            psy = chs[SERVO_Y_CH[i]]
            fa  = pwm_to_agent_force(pm, psx, psy)
            Fa  = math.sqrt(sum(v**2 for v in fa))
            exa = math.degrees(math.atan2(fa[0], fa[2])) if abs(fa[2]) > 0.01 else 0.0
            eya = math.degrees(math.atan2(fa[1], fa[2])) if abs(fa[2]) > 0.01 else 0.0
            print(f"  {lbl:>4} {pm:6d} {psx:6d} {psy:6d}  {Fa:6.3f}  {exa:+6.2f}° {eya:+6.2f}°")

        # 比較表
        dF = [F_act[k]-F_exp[k] for k in range(3)]
        dT = [T_act[k]-T_exp[k] for k in range(3)]
        print(f"\n  {'':6} │ {'Fx':>8} {'Fy':>8} {'Fz':>8} │ {'Tx':>8} {'Ty':>8} {'Tz':>8}")
        print("  ───────┼" + "─"*27 + "┼" + "─"*27)
        print(f"  {'期望':^6} │ {F_exp[0]:+8.3f} {F_exp[1]:+8.3f} {F_exp[2]:+8.3f} "
              f"│ {T_exp[0]:+8.3f} {T_exp[1]:+8.3f} {T_exp[2]:+8.3f}")
        print(f"  {'實際':^6} │ {F_act[0]:+8.3f} {F_act[1]:+8.3f} {F_act[2]:+8.3f} "
              f"│ {T_act[0]:+8.3f} {T_act[1]:+8.3f} {T_act[2]:+8.3f}")
        print(f"  {'誤差':^6} │ {dF[0]:+8.3f} {dF[1]:+8.3f} {dF[2]:+8.3f} "
              f"│ {dT[0]:+8.3f} {dT[1]:+8.3f} {dT[2]:+8.3f}")
    else:
        print("    （等待 /mavros/rc/out 數據...）")

    print("\n" + "═"*70)
    print("  [Ctrl-C 退出]  發布頻率: 20 Hz  顯示頻率: 2 Hz")
    print("═"*70)

# ════════════════════════════════════════════════════════════════
# Main
# ════════════════════════════════════════════════════════════════
def make_pos_target():
    msg = PositionTarget()
    msg.header.stamp       = rospy.Time.now()
    msg.coordinate_frame   = 8          # FRAME_BODY_NED（機體 FLU 速度）
    msg.type_mask          = MASK_VEL_ONLY
    msg.velocity.x         = VEL_FWD
    msg.velocity.y         = VEL_LEFT
    msg.velocity.z         = VEL_UP
    return msg

def main():
    rospy.init_node('vel_wrench_monitor')
    rospy.Subscriber('/mavros/imu/data', Imu,    cb_imu)
    rospy.Subscriber('/mavros/rc/out',   RCOut,  cb_rc)
    rospy.Subscriber('/mavros/state',    State,  cb_state)

    pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    rospy.loginfo("vel_wrench_monitor: 啟動")
    rospy.loginfo(f"  速度指令：fwd={VEL_FWD}  left={VEL_LEFT}  up={VEL_UP}  m/s")
    rospy.loginfo(f"  期望加速度（NWU）：{V_DOT_DES_NWU}  m/s²")

    rate = rospy.Rate(20)
    tick = 0
    while not rospy.is_shutdown():
        pub.publish(make_pos_target())
        tick += 1
        if tick % 10 == 0:
            display()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
