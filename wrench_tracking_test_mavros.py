#!/usr/bin/env python3
"""
Wrench Tracking Test (MAVROS / ROS 1 Noetic)
=============================================
給定期望 wrench 向量，讓無人機追蹤，並即時顯示估計的實際推力向量。

■ 座標系慣例（本腳本）
  body 系：FLU（Forward-Left-Up），與 ROS 標準相同
    +x = 機頭方向（前）
    +y = 左側
    +z = 上
  DESIRED_WRENCH = [Fx, Fy, Fz, Tx, Ty, Tz]（N, N·m）

■ 範例：[1, 1, 14, 0, 0, 0]
    Fx = 1 N（向前水平力）
    Fy = 1 N（向左水平力）
    Fz = 14 N（向上，≈ m*g，懸停所需垂直力）
    Tx = Ty = Tz = 0（無扭矩 → 機體維持水平）

■ 控制鏈
  PositionTarget (position hold + acc feedforward) → pfa_pos_control
    → vehicle_attitude_setpoint.thrust_body[3]
      → pfa_att_control → vehicle_thrust_setpoint / vehicle_torque_setpoint

■ 水平力注入方式
  透過 PositionTarget.acceleration_or_force（ENU frame）設定加速度前饋：
    acc_ff_ENU.x = (Fx*cos(yaw) - Fy*sin(yaw)) / m
    acc_ff_ENU.y = (Fx*sin(yaw) + Fy*cos(yaw)) / m
  （垂直力 Fz 由位置控制器自動維持高度，不額外注入）

■ 實際推力向量估計
  IMU 量測 body-frame 加速度（含重力反作用）：
    a_imu_body = a_body + R^T * g   → 即 F_body / m
  故 F_body_est = m * a_imu_body
  歸一化推力：t_norm = F_body_est / PFA_MAX_THR

■ 扭矩限制
  本腳本僅支援 Tx=Ty=Tz=0（無扭矩）。
  非零扭矩會造成機體持續旋轉，需另行設計安全保護機制。

■ 硬體限制
  水平力上限：|Fx| 或 |Fy| ≤ _THRUST_XY_MAX * PFA_MAX_THR = 0.3 * 24 = 7.2 N
  垂直力上限：Fz ≤ PFA_MAX_THR = 24 N
  來源：
    PFA_MAX_THR    = pfa_pos_control_params.c line 109
    PFA_MAX_TOR    = pfa_att_control_params.c  line 108
    _thrust_xy_max = pfa_pos_control.hpp        line 121

執行方式：
  python3 wrench_tracking_test_mavros.py
"""

import math
import threading
import rospy
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, ParamValue
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode,     SetModeRequest,
    ParamSet,    ParamSetRequest,
)

# ─────────────────────────────────────────────────────────────
# Configuration ← 使用者修改這裡
# ─────────────────────────────────────────────────────────────

# 期望 wrench 向量 [Fx, Fy, Fz, Tx, Ty, Tz]（N, N·m，body FLU 系）
DESIRED_WRENCH = [1.0, 1.0, 14.0, 0.0, 0.0, 0.0]

VEHICLE_MASS_KG  = 1.4     # 無人機質量（kg）：用於 force → acceleration 換算
TARGET_ALT_M     = 1.0     # 起飛後懸停高度（m）
TRACK_DURATION_S = 15.0    # 追蹤 wrench 的持續時間（s）
STOP_HOLD_S      = 3.0     # wrench 結束後位置保持時間（s）
CTRL_HZ          = 20      # Timer 控制頻率

# 韌體硬碼參數（請勿隨意更改，與韌體同步）
PFA_MAX_THR    = 24.0   # N  (pfa_pos_control_params.c line 109)
PFA_MAX_TOR    = 15.0   # N·m (pfa_att_control_params.c line 108)
THRUST_XY_MAX  = 0.3    # 水平歸一化推力上限 (pfa_pos_control.hpp line 121)

G = 9.81  # 重力加速度 (m/s²)

# ─────────────────────────────────────────────────────────────
# PositionTarget type_mask 常數
# ─────────────────────────────────────────────────────────────
# MASK_POS_HOLD：僅位置 + 偏航（速度 / 加速度全部忽略）
MASK_POS_HOLD = (8 + 16 + 32    # ignore vx vy vz
                + 64 + 128 + 256 # ignore ax ay az
                + 2048)          # ignore yaw_rate
# = 2552

# MASK_POS_ACC：位置保持 + 加速度前饋 + 偏航
# bit 9 = 512 → 將 acceleration_or_force 解讀為加速度（非力）
MASK_POS_ACC = (8 + 16 + 32     # ignore vx vy vz
               + 512             # type = acceleration
               + 2048)           # ignore yaw_rate
# = 2616

# ─────────────────────────────────────────────────────────────
# 啟動前驗證
# ─────────────────────────────────────────────────────────────
def _validate_wrench(w):
    Fx, Fy, Fz, Tx, Ty, Tz = w
    ok = True
    xy_limit = THRUST_XY_MAX * PFA_MAX_THR  # 7.2 N

    if abs(Fx) > xy_limit:
        print(f"[WARN] Fx={Fx:.2f} N 超過水平力上限 {xy_limit:.1f} N，韌體將飽和截斷")
        ok = False
    if abs(Fy) > xy_limit:
        print(f"[WARN] Fy={Fy:.2f} N 超過水平力上限 {xy_limit:.1f} N，韌體將飽和截斷")
        ok = False
    if Fz <= 0 or Fz > PFA_MAX_THR:
        print(f"[WARN] Fz={Fz:.2f} N 超出有效範圍 (0, {PFA_MAX_THR}]")
        ok = False
    if abs(Fz - VEHICLE_MASS_KG * G) > VEHICLE_MASS_KG * G * 0.5:
        print(f"[WARN] Fz={Fz:.2f} N 與懸停力 {VEHICLE_MASS_KG*G:.2f} N 相差超過 50%，"
              f"可能導致飛行高度不穩")
    if any(abs(t) > 0.01 for t in [Tx, Ty, Tz]):
        print(f"[WARN] 本腳本僅安全支援 Tx=Ty=Tz=0（無扭矩）")
        print(f"       非零扭矩將導致機體持續旋轉！請確認你了解後果再繼續。")
        ok = False

    # 顯示歸一化推力預估
    fx_n = Fx / PFA_MAX_THR
    fy_n = Fy / PFA_MAX_THR
    fz_n = Fz / PFA_MAX_THR
    print(f"\n[INFO] 期望 wrench：Fx={Fx:.2f} Fy={Fy:.2f} Fz={Fz:.2f} | "
          f"Tx={Tx:.2f} Ty={Ty:.2f} Tz={Tz:.2f}")
    print(f"[INFO] 歸一化推力估算：[{fx_n:.3f}, {fy_n:.3f}, {fz_n:.3f}]  "
          f"（上限：xy≤{THRUST_XY_MAX:.1f}, z≤1.0）")
    return ok

# ─────────────────────────────────────────────────────────────
# Subscribers
# ─────────────────────────────────────────────────────────────
_state = State()
_pose  = PoseStamped()
_imu   = Imu()

def _cb_state(msg): global _state; _state = msg
def _cb_pose(msg):  global _pose;  _pose  = msg
def _cb_imu(msg):   global _imu;   _imu   = msg

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

def get_imu_force_body():
    """
    IMU linear_acceleration 為 body FLU 座標，已含重力反作用（pseudo-acceleration）。
    F_body_est = m * a_imu_body ≈ 實際推力向量（body FLU，N）
    靜態懸停時：Fz_est ≈ m * g
    """
    a = _imu.linear_acceleration
    return (VEHICLE_MASS_KG * a.x,
            VEHICLE_MASS_KG * a.y,
            VEHICLE_MASS_KG * a.z)

# ─────────────────────────────────────────────────────────────
# Shared setpoint（Timer 背景發布）
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
# PositionTarget 工廠函式
# ─────────────────────────────────────────────────────────────
def make_pos_sp(x, y, z, yaw=None):
    """純位置保持 setpoint（無加速度前饋）。"""
    msg = PositionTarget()
    msg.header.frame_id  = 'map'
    msg.coordinate_frame = 1
    msg.type_mask        = MASK_POS_HOLD
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    msg.yaw = yaw if yaw is not None else get_yaw_rad()
    return msg

def make_wrench_sp(hold_x, hold_y, hold_z, Fx, Fy, yaw_hold):
    """
    位置保持 + 水平力前饋 setpoint。
    Fx, Fy：body FLU 水平力（N），透過加速度前饋注入。
    垂直力 Fz 由位置控制器自動提供（維持 hold_z 高度）。
    """
    yaw = get_yaw_rad()

    # Body FLU [Fx, Fy] → ENU [ax, ay]
    ax_enu = (Fx * math.cos(yaw) - Fy * math.sin(yaw)) / VEHICLE_MASS_KG
    ay_enu = (Fx * math.sin(yaw) + Fy * math.cos(yaw)) / VEHICLE_MASS_KG

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
    msg.acceleration_or_force.z = 0.0  # Fz 由位置控制器提供
    return msg

# ─────────────────────────────────────────────────────────────
# 服務幫助函式
# ─────────────────────────────────────────────────────────────
def param_set(name, value, retries=5):
    try:
        rospy.wait_for_service('/mavros/param/set', timeout=5)
        svc = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        for _ in range(retries):
            if svc(req).success:
                return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"param_set({name}): {e}")
    return False

def set_mode(mode_str, retries=5):
    try:
        rospy.wait_for_service('/mavros/set_mode', timeout=5)
        svc = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        for _ in range(retries):
            if svc(SetModeRequest(custom_mode=mode_str)).mode_sent:
                return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"set_mode({mode_str}): {e}")
    return False

def arm(armed=True, retries=5):
    try:
        rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
        svc = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        for _ in range(retries):
            if svc(CommandBoolRequest(value=armed)).success:
                return True
            rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"arm({armed}): {e}")
    return False

def wait_for_alt(target_z, tol=0.10, stable_s=2.0, timeout_s=30.0):
    """等到高度在 target_z ± tol 內連續 stable_s 秒。"""
    stable_since = None
    t0 = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout_s:
            rospy.logwarn("wait_for_alt: timeout!")
            return False
        err = abs(get_alt() - target_z)
        if err < tol:
            if stable_since is None:
                stable_since = rospy.Time.now()
            elif (rospy.Time.now() - stable_since).to_sec() >= stable_s:
                return True
        else:
            stable_since = None
        rate.sleep()
    return False

# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main():
    global sp_pub

    rospy.init_node('wrench_tracking_test')
    rate = rospy.Rate(CTRL_HZ)

    # 解析 wrench
    Fx, Fy, Fz, Tx, Ty, Tz = DESIRED_WRENCH
    if not _validate_wrench(DESIRED_WRENCH):
        print("[ERROR] Wrench 參數超出安全限制，請修正後重試。")
        return

    # Subscribers
    rospy.Subscriber('/mavros/state',                    State,        _cb_state)
    rospy.Subscriber('/mavros/local_position/pose',      PoseStamped,  _cb_pose)
    rospy.Subscriber('/mavros/imu/data',                 Imu,          _cb_imu)

    # Publisher + Timer
    sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    _timer = rospy.Timer(rospy.Duration(1.0 / CTRL_HZ), _timer_cb)

    rospy.sleep(1.0)

    # ── Step 1：起飛前預先發 setpoint（避免 COM_OF_LOSS_T 超時）──
    print("\n[1] 預熱 setpoint 發布...")
    x0 = _pose.pose.position.x
    y0 = _pose.pose.position.y
    yaw0 = get_yaw_rad()
    set_sp(make_pos_sp(x0, y0, TARGET_ALT_M, yaw0))
    rospy.sleep(2.0)

    # ── Step 2：切換 OFFBOARD + 解鎖 ──
    print("[2] 切換 OFFBOARD 模式...")
    if not set_mode('OFFBOARD'):
        rospy.logerr("OFFBOARD 切換失敗"); return
    rospy.sleep(0.3)

    print("[2] 解鎖...")
    if not arm(True):
        rospy.logerr("解鎖失敗"); return

    # ── Step 3：爬升至目標高度 ──
    print(f"[3] 爬升至 {TARGET_ALT_M:.1f} m...")
    if not wait_for_alt(TARGET_ALT_M, tol=0.12, stable_s=2.0, timeout_s=30.0):
        rospy.logwarn("爬升超時，繼續執行...")

    # 鎖定懸停位置
    hold_x = _pose.pose.position.x
    hold_y = _pose.pose.position.y
    hold_z = TARGET_ALT_M
    yaw_hold = get_yaw_rad()
    print(f"[3] 懸停位置鎖定：x={hold_x:.2f} y={hold_y:.2f} z={hold_z:.2f} yaw={math.degrees(yaw_hold):.1f}°")

    # 設定 PFA_DES_ROLL / PITCH = 0（確保機體水平，僅靠推力向量傾斜提供水平力）
    print("[3] 設定 PFA_DES_ROLL=0, PFA_DES_PITCH=0...")
    param_set('PFA_DES_ROLL',  0.0)
    param_set('PFA_DES_PITCH', 0.0)

    rospy.sleep(1.0)

    # ── Step 4：追蹤期望 wrench ──
    print(f"\n[4] 開始追蹤 wrench [{Fx:.2f}, {Fy:.2f}, {Fz:.2f}, {Tx:.2f}, {Ty:.2f}, {Tz:.2f}]")
    print(f"    持續 {TRACK_DURATION_S:.0f} 秒")
    print(f"\n{'Time':>5} | {'Cmd Fx':>7} {'Cmd Fy':>7} {'Cmd Fz':>7} "
          f"| {'Est Fx':>7} {'Est Fy':>7} {'Est Fz':>7} "
          f"| {'|t_norm|':>8} | {'Roll':>6} {'Pitch':>6} {'Alt':>5}")
    print("-" * 95)

    t_track = rospy.Time.now()
    log_timer = rospy.Time.now()
    LOG_INTERVAL = 0.5  # 每 0.5 秒印一次

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - t_track).to_sec()
        if elapsed >= TRACK_DURATION_S:
            break

        # 計算 wrench sp → PositionTarget
        sp = make_wrench_sp(hold_x, hold_y, hold_z, Fx, Fy, yaw_hold)
        set_sp(sp)

        # 日誌
        if (rospy.Time.now() - log_timer).to_sec() >= LOG_INTERVAL:
            log_timer = rospy.Time.now()
            fx_e, fy_e, fz_e = get_imu_force_body()
            t_norm = math.sqrt((fx_e/PFA_MAX_THR)**2 +
                               (fy_e/PFA_MAX_THR)**2 +
                               (fz_e/PFA_MAX_THR)**2)
            roll_d, pitch_d, yaw_d = get_rpy_deg()
            print(f"{elapsed:5.1f} | {Fx:7.2f} {Fy:7.2f} {Fz:7.2f} "
                  f"| {fx_e:7.2f} {fy_e:7.2f} {fz_e:7.2f} "
                  f"| {t_norm:8.4f} "
                  f"| {roll_d:6.1f} {pitch_d:6.1f} {get_alt():5.2f}")

        rate.sleep()

    # ── Step 5：移除水平力前饋，回到純位置保持 ──
    print("\n[5] 移除水平力前饋，純位置保持...")
    set_sp(make_pos_sp(hold_x, hold_y, hold_z, yaw_hold))
    rospy.sleep(STOP_HOLD_S)

    # ── Step 6：Landing ──
    print("[6] AUTO.LAND...")
    set_mode('AUTO.LAND')
    rospy.sleep(5.0)

    # ── 結果摘要 ──
    fx_e, fy_e, fz_e = get_imu_force_body()
    t_norm = math.sqrt((fx_e/PFA_MAX_THR)**2 +
                       (fy_e/PFA_MAX_THR)**2 +
                       (fz_e/PFA_MAX_THR)**2)
    print("\n" + "="*60)
    print("追蹤結束摘要")
    print("="*60)
    print(f"  期望 wrench    : Fx={Fx:.2f} Fy={Fy:.2f} Fz={Fz:.2f} N")
    print(f"  末次估計 F_body: Fx={fx_e:.2f} Fy={fy_e:.2f} Fz={fz_e:.2f} N")
    print(f"  歸一化推力向量 : [{fx_e/PFA_MAX_THR:.3f}, {fy_e/PFA_MAX_THR:.3f}, "
          f"{fz_e/PFA_MAX_THR:.3f}]  |t|={t_norm:.4f}")
    print(f"  水平力上限     : {THRUST_XY_MAX:.1f} × {PFA_MAX_THR:.1f} = "
          f"{THRUST_XY_MAX*PFA_MAX_THR:.1f} N")
    print("="*60)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
