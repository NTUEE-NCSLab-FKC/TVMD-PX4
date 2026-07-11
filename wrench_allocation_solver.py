#!/usr/bin/env python3
"""
TVMD Wrench → Per-Agent Thrust Allocation Solver
=================================================
給定期望 wrench [Fx, Fy, Fz, Tx, Ty, Tz]，
計算各 agent 的推力向量、推力大小與傾斜角度。

■ 速度 / 角速度 → 期望 wrench（Newton-Euler 逆動力學）
  給定 v_des（3D 線速度）和 ω_des（3D 角速度）：
    F_body = Rᵀ · (m·v̇_des + m·g_world)      ← 線性動力學
    τ_body = I·ω̇_des + ω × (I·ω)             ← 旋轉動力學

  常見情況：
    等速飛行（v̇=0）  → F_z = m·g（純懸停），F_x=F_y=0
    前向加速       → F_x += m·a_x（慣性力）
    偏航（ω̇=0）   → τ ≈ ω × (I·ω)（陀螺項，慢轉時≈0）

■ 完整控制鏈
  DESIRED_WRENCH [Fx, Fy, Fz, Tx, Ty, Tz]
    ↓ [pfa_pos_control]
      F_body = m*(a_ff + g + Kv*err_vel + Kp*err_pos)
      thrust_body = F_body / PFA_MAX_THR          →  vehicle_attitude_setpoint
    ↓ [pfa_att_control]
      τ = -I*(Kr*e_R + Kω*err_ω) / PFA_MAX_TOR
      → vehicle_thrust_setpoint [fx,fy,fz] （歸一化）
      → vehicle_torque_setpoint [τx,τy,τz]（歸一化）
    ↓ [control_allocator / ActuatorEffectivenessVTOL_TVMD]
      有效性矩陣 B（6×3n）：
        B = [ p̂₀·Rz₀  p̂₁·Rz₁  ... ]  列 0-2：扭矩（單位 m·N/N = m）
            [   Rz₀     Rz₁    ... ]  列 3-5：力（N/N = 1）
      偽逆分配：u_local = B⁺ · W（最小範數解）
    ↓ 各 agent i
      f_i    = [fxi, fyi, fzi]           機體系力向量（N）
      F_i    = ‖f_i‖                     馬達推力（N）
      η_xi   = atan2(fxi, fzi)           X 傾斜角（deg）
      η_yi   = atan2(fyi, fzi)           Y 傾斜角（deg）
      u_prop = (F_i - Tf0) / c_l        馬達油門 [0,1]

■ 座標系：機體 NWU（North-West-Up），pfa_att_control 內部使用
    +Fx = 前進（Forward）  +Tx = 右滾力矩（向右傾）
    +Fy = 左側（Left）     +Ty = 仰頭力矩（抬頭）
    +Fz = 向上（Up）       +Tz = 左偏力矩（左偏航）
  懸停：Fz > 0，如 Fz ≈ m*g = 13.7 N
  位置參數驗證：CA_MD0_PY=0.16 → Front-LEFT ✓（NWU +y = left）

執行：  python3 wrench_allocation_solver.py
"""

import math

# ─────────────────────────────────────────────────────────────
# Configuration ← 使用者修改這裡
# ─────────────────────────────────────────────────────────────
# 模式 A：直接給 wrench（N, N·m），機體 NWU 系
# 模式 B：給速度 + 角速度，自動計算期望 wrench
USE_VELOCITY_INPUT = True   # True = 模式 B，False = 模式 A

# ── 模式 A：期望 wrench ──
# 懸停：Fz ≈ m*g = 13.7 N（NWU 向上 = 正）
DESIRED_WRENCH = [1.0, 1.0, 14.0, 0.0, 0.0, 0.0]

# ── 模式 B：速度 + 角速度 → 自動推導 wrench ──
# 線速度（NWU：+x=前, +y=左, +z=上）單位：m/s
V_DES     = [0.5, 0.0, 0.0]   # 期望線速度（等速時不需額外水平力）
V_DOT_DES = [0.5, 0.0, 0.0]   # 期望線加速度（m/s²，=0 表等速）

# 角速度（NWU：+x=右滾, +y=仰頭, +z=左偏航）單位：rad/s
W_DES     = [0.0, 0.0, 0.0]   # 期望角速度
W_DOT_DES = [0.0, 0.0, 0.0]   # 期望角加速度（rad/s²）
W_CURRENT = [0.0, 0.0, 0.0]   # 當前角速度（用於陀螺項計算）

# 當前姿態（用於重力旋轉到機體系）
ROLL_DEG  = 0.0   # deg（NWU：右翼下壓 = 正）
PITCH_DEG = 0.0   # deg（NWU：抬頭 = 正）
YAW_DEG   = 0.0   # deg（NWU：左偏 = 正）

# 慣性矩陣（kg·m²，NWU 機體系，對角近似）
# 來源：VEH_AGENT_IXX / IYY / IZZ 參數
I_XX = 0.015   # Roll 軸
I_YY = 0.015   # Pitch 軸
I_ZZ = 0.025   # Yaw 軸

# ─────────────────────────────────────────────────────────────
# 韌體與車體參數（對應 13300_generic_vtol_tvmd）
# ─────────────────────────────────────────────────────────────
PFA_MAX_THR = 24.0    # N    (pfa_pos_control_params.c line 109)
PFA_MAX_TOR = 15.0    # N·m  (pfa_att_control_params.c line 108)
VEHICLE_MASS = 1.4    # kg
G = 9.81              # m/s²

# 馬達推力映射：T = c_l * u_prop + Tf0
C_L  = 6.125    # N (ActuatorEffectivenessVTOL_TVMD.hpp line 69)
TF0  = 0.0      # bias

SERVO_MAX_DEG = 30.0   # ±30°（CA_SV_TL*_MAXA）

# TVMD 幾何（H-frame 4 agents，FRD 座標）
# (px, py, pz, az_psi, label)
# az_psi: 0 → Rz(0°)=I, 1 → Rz(90°)（目前全為 0）
AGENTS = [
    ( 0.16,  0.16,  0.0,  0,  'Front-Left  (Agent 0)'),
    ( 0.16, -0.16,  0.0,  0,  'Front-Right (Agent 1)'),
    (-0.16,  0.16,  0.0,  0,  'Rear-Left   (Agent 2)'),
    (-0.16, -0.16,  0.0,  0,  'Rear-Right  (Agent 3)'),
]

# ─────────────────────────────────────────────────────────────
# 純 Python 矩陣工具（無 numpy 依賴）
# 矩陣以 list of rows 表示：M[row][col]
# ─────────────────────────────────────────────────────────────
def mat_zero(r, c):
    return [[0.0]*c for _ in range(r)]

def mat_T(M):
    r, c = len(M), len(M[0])
    return [[M[i][j] for i in range(r)] for j in range(c)]

def mat_mul(A, B):
    rA, cA = len(A), len(A[0])
    rB, cB = len(B), len(B[0])
    assert cA == rB
    C = mat_zero(rA, cB)
    for i in range(rA):
        for j in range(cB):
            for k in range(cA):
                C[i][j] += A[i][k] * B[k][j]
    return C

def mat_inv(M):
    """n×n 矩陣逆（Gauss-Jordan）。"""
    n = len(M)
    aug = [row[:] + [1.0 if i == j else 0.0 for j in range(n)]
           for i, row in enumerate(M)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        aug[col], aug[pivot] = aug[pivot], aug[col]
        if abs(aug[col][col]) < 1e-12:
            raise ValueError("Matrix is singular or near-singular")
        scale = aug[col][col]
        aug[col] = [v / scale for v in aug[col]]
        for row in range(n):
            if row != col:
                f = aug[row][col]
                aug[row] = [aug[row][k] - f * aug[col][k] for k in range(2*n)]
    return [row[n:] for row in aug]

def mat_pinv_right(B):
    """
    右偽逆：B⁺ = Bᵀ(BBᵀ)⁻¹  （適用於 rows < cols 的矩陣）
    """
    Bt = mat_T(B)
    BBt = mat_mul(B, Bt)
    BBt_inv = mat_inv(BBt)
    return mat_mul(Bt, BBt_inv)

def mv_mul(M, v):
    """矩陣乘向量。"""
    return [sum(M[i][j]*v[j] for j in range(len(v))) for i in range(len(M))]

def hat_mat(v):
    """3D 向量的 skew-symmetric 矩陣 v̂。"""
    return [
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ],
    ]

def cross3(a, b):
    """3D 向量外積 a × b。"""
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]

def euler_to_R(roll_rad, pitch_rad, yaw_rad):
    """ZYX Euler 角 → 旋轉矩陣 R（機體 NWU → 世界 NWU）。"""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return [
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ],
    ]

def velocity_to_wrench(v_dot_des, w_dot_des, w_current,
                       roll_rad, pitch_rad, yaw_rad,
                       mass, i_xx, i_yy, i_zz, g=9.81):
    """
    Newton-Euler 逆動力學：由期望加速度計算所需 wrench（機體 NWU 系）。

    線性：F_body = Rᵀ · (m·v̇_des_world + m·[0,0,g])
        - g_world=[0,0,-g]（NWU，重力向下）
        - F_ext_world = m·v̇_des - m·g_world = m·v̇_des + m·[0,0,g]
    旋轉：τ_body = I·ω̇_des + ω_current × (I·ω_current)
    """
    R = euler_to_R(roll_rad, pitch_rad, yaw_rad)
    Rt = mat_T(R)

    # 世界系所需推力（抵消重力 + 提供期望加速度）
    F_world = [
        mass * v_dot_des[0],
        mass * v_dot_des[1],
        mass * v_dot_des[2] + mass * g,
    ]
    F_body = mv_mul(Rt, F_world)

    # 陀螺項：ω × (I·ω)
    I_diag = [i_xx, i_yy, i_zz]
    Iw = [I_diag[j] * w_current[j] for j in range(3)]
    gyro = cross3(w_current, Iw)

    # 所需力矩：τ = I·ω̇ + ω×(I·ω)
    tau_body = [I_diag[j] * w_dot_des[j] + gyro[j] for j in range(3)]

    return F_body, tau_body

def Rz_mat(psi_rad):
    c, s = math.cos(psi_rad), math.sin(psi_rad)
    return [
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ]

# ─────────────────────────────────────────────────────────────
# 有效性矩陣 B（6 × 3n）
# ─────────────────────────────────────────────────────────────
def build_B(agents):
    n = len(agents)
    B = mat_zero(6, 3 * n)
    for i, (px, py, pz, az_psi, _) in enumerate(agents):
        p = [px, py, pz]
        R = Rz_mat(math.pi / 2 * az_psi)
        phat = hat_mat(p)
        phatR = mat_mul(phat, R)   # 3×3

        for r in range(3):
            for c in range(3):
                B[r][3*i + c] = phatR[r][c]    # 扭矩
                B[r+3][3*i + c] = R[r][c]      # 力
    return B

def allocate(B, W):
    """f_vec = B⁺ · W（最小範數解）。"""
    Bpinv = mat_pinv_right(B)
    return mv_mul(Bpinv, W)

def verify(B, f_vec, W):
    W_rec = mv_mul(B, f_vec)
    err = [abs(W_rec[k] - W[k]) for k in range(len(W))]
    return W_rec, err

# ─────────────────────────────────────────────────────────────
# 各 agent 結果解析
# ─────────────────────────────────────────────────────────────
def parse_agent(f3):
    fx, fy, fz = f3
    F = math.sqrt(fx**2 + fy**2 + fz**2)
    # NWU：主推力方向 = +z（向上）
    # η_x：從 +z 軸向 +x 軸傾斜（前進方向）
    # η_y：從 +z 軸向 +y 軸傾斜（左側方向）
    eta_x = math.degrees(math.atan2(fx, fz)) if abs(fz) > 1e-9 else math.copysign(90.0, fx)
    eta_y = math.degrees(math.atan2(fy, fz)) if abs(fz) > 1e-9 else math.copysign(90.0, fy)
    u_prop = max(0.0, min(1.0, (F - TF0) / C_L))
    return F, eta_x, eta_y, u_prop

# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main():
    print("=" * 72)
    print("  TVMD Wrench → Per-Agent Allocation Solver")
    print("=" * 72)

    if USE_VELOCITY_INPUT:
        # ── 模式 B：速度 / 角速度 → 期望 wrench ──
        roll_r  = math.radians(ROLL_DEG)
        pitch_r = math.radians(PITCH_DEG)
        yaw_r   = math.radians(YAW_DEG)

        F_body, tau_body = velocity_to_wrench(
            V_DOT_DES, W_DOT_DES, W_CURRENT,
            roll_r, pitch_r, yaw_r,
            VEHICLE_MASS, I_XX, I_YY, I_ZZ, G,
        )
        Fx, Fy, Fz = F_body
        Tx, Ty, Tz = tau_body

        print(f"\n模式 B：速度 / 角速度 → 期望 wrench（Newton-Euler 逆動力學）")
        print(f"  當前姿態：roll={ROLL_DEG:.1f}° pitch={PITCH_DEG:.1f}° yaw={YAW_DEG:.1f}°")
        print(f"  期望線速度  v_des     = {V_DES}")
        print(f"  期望線加速度 v_dot_des = {V_DOT_DES}  m/s²")
        print(f"  期望角速度  w_des     = {W_DES}  rad/s")
        print(f"  期望角加速度 w_dot_des = {W_DOT_DES}  rad/s²")
        print(f"  當前角速度  w_current = {W_CURRENT}  rad/s")
        print(f"\n  → 計算所需 wrench（機體 NWU）：")
        print(f"      力  ：Fx={Fx:+8.3f} N    Fy={Fy:+8.3f} N    Fz={Fz:+8.3f} N")
        print(f"      力矩：Tx={Tx:+8.3f} N·m  Ty={Ty:+8.3f} N·m  Tz={Tz:+8.3f} N·m")
    else:
        # ── 模式 A：直接給 wrench ──
        Fx, Fy, Fz, Tx, Ty, Tz = DESIRED_WRENCH
        print(f"\n模式 A：直接給定期望 wrench（機體 NWU）：")
        print(f"  力  ：Fx={Fx:+8.3f} N    Fy={Fy:+8.3f} N    Fz={Fz:+8.3f} N")
        print(f"  力矩：Tx={Tx:+8.3f} N·m  Ty={Ty:+8.3f} N·m  Tz={Tz:+8.3f} N·m")

    print(f"  懸停參考：Fz ≈ {VEHICLE_MASS*G:.2f} N（NWU 向上 = 正）")

    # 韌體內部順序：[τx, τy, τz, fx, fy, fz]
    W = [Tx, Ty, Tz, Fx, Fy, Fz]

    # ── 建有效性矩陣 ──
    B = build_B(AGENTS)

    print(f"\n有效性矩陣 B（6 × {3*len(AGENTS)}）— 列=[Tx Ty Tz Fx Fy Fz], 行=[fx0 fy0 fz0 ...]：")
    row_lbl = ['Tx','Ty','Tz','Fx','Fy','Fz']
    for r, row in enumerate(B):
        vals = " ".join(f"{v:+7.4f}" for v in row)
        print(f"  {row_lbl[r]}: [{vals}]")

    # ── 偽逆分配 ──
    f_vec = allocate(B, W)

    print(f"\n偽逆分配 f_vec（各 agent 局部力向量）：")
    for i in range(len(AGENTS)):
        f = f_vec[3*i:3*i+3]
        print(f"  Agent {i}: [{f[0]:+8.3f}, {f[1]:+8.3f}, {f[2]:+8.3f}] N")

    # ── 驗證 ──
    W_rec, err = verify(B, f_vec, W)
    print(f"\n驗證（B·f_vec ↔ 期望 W）：")
    for k, lbl in enumerate(['Tx','Ty','Tz','Fx','Fy','Fz']):
        print(f"  {lbl}：期望={W[k]:+8.3f}  重建={W_rec[k]:+8.3f}  誤差={err[k]:.2e}")

    # ── 各 agent 詳細輸出 ──
    print(f"\n{'='*72}")
    print(f"  各 Agent 推力分解結果")
    print(f"{'='*72}")
    hdr = (f"{'Agent':<24} {'fx':>7} {'fy':>7} {'fz':>7}  "
           f"{'F_motor':>8}  {'η_x':>7} {'η_y':>7}  {'u_prop':>7}  狀態")
    print(hdr)
    print("-" * 78)

    all_ok = True
    for i, (_, _, _, _, label) in enumerate(AGENTS):
        f3 = f_vec[3*i:3*i+3]
        fx, fy, fz = f3
        F, eta_x, eta_y, u_prop = parse_agent(f3)

        warns = []
        if abs(eta_x) > SERVO_MAX_DEG: warns.append(f"η_x超{SERVO_MAX_DEG:.0f}°")
        if abs(eta_y) > SERVO_MAX_DEG: warns.append(f"η_y超{SERVO_MAX_DEG:.0f}°")
        if u_prop >= 0.999:            warns.append("油門飽和")
        if F < 0.01:                   warns.append("推力≈0")
        status = "✓" if not warns else "✗ " + " ".join(warns)
        if warns: all_ok = False

        print(f"{label:<24} {fx:+7.3f} {fy:+7.3f} {fz:+7.3f}  "
              f"{F:8.3f} N  {eta_x:+7.2f}° {eta_y:+7.2f}°  {u_prop:7.4f}  {status}")

    total = sum(parse_agent(f_vec[3*i:3*i+3])[0] for i in range(len(AGENTS)))
    print("-" * 78)
    print(f"{'合計':24} {'':>23}  {total:8.3f} N")

    # ── 歸一化 wrench（韌體輸出值）──
    # 對應 pfa_pos_control 發出的 thrust_body（NED/FRD 歸一化）：
    # pfa_att_control 內部 NWU↔FRD 轉換：NWU_x=FRD_x, NWU_y=-FRD_y, NWU_z=-FRD_z
    # 因此 thrust_body_FRD = [Fx/THR, -Fy/THR, -Fz/THR]
    print(f"\n歸一化 wrench（vehicle_thrust/torque_setpoint，[-1,1] 範圍）：")
    print(f"  torque_norm [NWU] = [{Tx/PFA_MAX_TOR:+.4f}, {Ty/PFA_MAX_TOR:+.4f}, {Tz/PFA_MAX_TOR:+.4f}]")
    print(f"  thrust_norm [NWU] = [{Fx/PFA_MAX_THR:+.4f}, {Fy/PFA_MAX_THR:+.4f}, {Fz/PFA_MAX_THR:+.4f}]")
    print(f"  thrust_body [FRD] = [{Fx/PFA_MAX_THR:+.4f}, {-Fy/PFA_MAX_THR:+.4f}, {-Fz/PFA_MAX_THR:+.4f}]  ← attitude_setpoint.thrust_body")
    print(f"  韌體約束（FRD）：xy ∈ [-{0.3},{0.3}], z ∈ [-1,0]")

    print(f"\n整體可行性：{'✓ 所有 agent 在物理限制內' if all_ok else '✗ 部分 agent 超出限制'}")
    print(f"  Servo 限制 = ±{SERVO_MAX_DEG}°  |  c_l = {C_L} N/throttle  |  PFA_MAX_THR = {PFA_MAX_THR} N")
    print(f"\n座標系說明（NWU：North-West-Up）：")
    print(f"  η_x > 0：推力向量向前（+North）傾斜")
    print(f"  η_y > 0：推力向量向左（+West）傾斜")
    print("=" * 72)

if __name__ == '__main__':
    main()
