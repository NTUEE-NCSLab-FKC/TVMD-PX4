#!/usr/bin/env python3
"""
PFA Roll Forward (MAVROS2 / ROS 2)
=====================================
Task: 直線前進，roll 從 +45° 到 -45° 來回振盪（正弦波）

軌跡 (ENU，yaw=0° 朝東)：
  x(t) = x₀ + t × PATH_SPEED_MPS   (朝東前進)
  y    = 0
  z    = TARGET_ALT_M
  yaw  = 0°

PFA_DES_ROLL(t) = ROLL_AMP × sin(2π t / ROLL_PERIOD_S)
  t=0    →  0°     t=T/4 → +45°    t=T/2 →  0°
  t=3T/4 → -45°    t=T   →  0°   (一圈完成)

ROS 2 重點：
  - rospy.Timer 在 ROS 1 是獨立執行緒；ROS 2 改用 rclpy.spin_once()
    在主迴圈中保持 callback 處理
  - param service 以 call_async + spin_until_future_complete 呼叫
  - /mavros/param/set 每 0.2 s 更新一次（~5 Hz）

執行方式：
  python3 pfa_roll_forward_ros2.py
"""

import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M    = 1.0    # 懸停高度 (m，ENU z+)
PATH_SPEED_MPS  = 0.5    # 前進速度 (m/s，ENU x+)
ROLL_AMP_DEG    = 45.0   # roll 振盪幅度 (deg)；飽和限制 ≈ ±31.6°
ROLL_PERIOD_S   = 6.0    # 振盪週期 (s)
NUM_CYCLES      = 3      # 週期數

HOVER_PHASE_DUR   = 8.0
ALT_TOL           = 0.15
HOVER_STABLE_TIME = 3.0
CTRL_HZ           = 20

# ─────────────────────────────────────────────────────────────
# Derived
# ─────────────────────────────────────────────────────────────
TRACK_DUR_S  = NUM_CYCLES * ROLL_PERIOD_S
TOTAL_PATH_M = PATH_SPEED_MPS * TRACK_DUR_S
OMEGA        = 2.0 * math.pi / ROLL_PERIOD_S

_HOVER_THR  = (1.4 * 9.81) / 24.0
_SAT_LIMIT  = math.degrees(math.asin(0.3 / _HOVER_THR))


# ─────────────────────────────────────────────────────────────
# Quaternion helpers
# ─────────────────────────────────────────────────────────────
def quat_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw*0.5),   math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5),  math.sin(roll*0.5)
    return (sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy)


def euler_from_quat(x, y, z, w):
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sp    = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(sp)
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────
class PFARollForwardNode(Node):

    def __init__(self):
        super().__init__('pfa_roll_forward')

        reliable_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                  history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        sensor_qos   = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                  history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(State,       '/mavros/state',               self._cb_state, reliable_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._cb_pose,  sensor_qos)
        self.create_subscription(Imu,         '/mavros/imu/data',            self._cb_imu,   sensor_qos)

        self._sp_pub    = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', reliable_qos)
        self._arm_cli   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_cli  = self.create_client(SetMode,     '/mavros/set_mode')
        self._param_cli = self.create_client(ParamSet,    '/mavros/param/set')

        self._state = State()
        self._pose  = PoseStamped()
        self._imu   = Imu()

    def _cb_state(self, msg): self._state = msg
    def _cb_pose(self, msg):  self._pose  = msg
    def _cb_imu(self, msg):   self._imu   = msg

    def altitude(self):
        return self._pose.pose.position.z

    def position_xy(self):
        return self._pose.pose.position.x, self._pose.pose.position.y

    def roll_deg(self):
        o = self._imu.orientation
        r, _, _ = euler_from_quat(o.x, o.y, o.z, o.w)
        return math.degrees(r)

    def _make_sp(self, x=0.0, y=0.0, z=TARGET_ALT_M, yaw=0.0):
        sp = PoseStamped()
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        qx, qy, qz, qw = quat_from_euler(0.0, 0.0, yaw)
        sp.pose.orientation.x = qx
        sp.pose.orientation.y = qy
        sp.pose.orientation.z = qz
        sp.pose.orientation.w = qw
        return sp

    def _elapsed(self, t0):
        return (self.get_clock().now() - t0).nanoseconds / 1e9

    def _pub_and_spin(self, sp=None):
        """Publish setpoint and process one ROS callback batch."""
        self._sp_pub.publish(sp or self._make_sp())
        rclpy.spin_once(self, timeout_sec=0.05)

    # ── Services ─────────────────────────────────────────────
    def param_set(self, name, value, retries=3):
        if not self._param_cli.wait_for_service(timeout_sec=5.0):
            return False
        req = ParamSet.Request()
        req.param_id = name
        req.value    = ParamValue(integer=0, real=float(value))
        for _ in range(retries):
            future = self._param_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() is not None and future.result().success:
                return True
        return False

    def set_mode(self, mode):
        if not self._mode_cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().mode_sent

    def arm_vehicle(self, do_arm=True):
        if not self._arm_cli.wait_for_service(timeout_sec=5.0):
            return False
        req = CommandBool.Request()
        req.value = do_arm
        future = self._arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success

    # ── Main task ────────────────────────────────────────────
    def run(self):
        log = self.get_logger()
        log.info(f"Roll: ±{ROLL_AMP_DEG:.0f}° × {NUM_CYCLES} cycles  "
                 f"({'OK' if ROLL_AMP_DEG <= _SAT_LIMIT else f'WARNING > {_SAT_LIMIT:.1f}°'})")
        log.info(f"Path: {PATH_SPEED_MPS} m/s × {TRACK_DUR_S:.0f} s = {TOTAL_PATH_M:.1f} m (ENU East)")

        # FCU connection
        log.info("Waiting for FCU connection...")
        while not self._state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
        log.info(f"  Connected. mode={self._state.mode}")

        # ── Step 0 ───────────────────────────────────────────
        log.info("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0°...")
        log.info("  PFA_DES_ROLL  = 0° ... " + ("OK" if self.param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
        log.info("  PFA_DES_PITCH = 0° ... " + ("OK" if self.param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

        # ── Step 1: Stream setpoints ─────────────────────────
        log.info(f"\n[Step 1] Streaming setpoints (5 s)...")
        t0 = self.get_clock().now()
        while self._elapsed(t0) < 5.0:
            self._pub_and_spin()

        # ── Step 2: OFFBOARD + Arm ────────────────────────────
        log.info("  Requesting OFFBOARD mode...")
        self.set_mode('OFFBOARD')

        t0 = self.get_clock().now()
        while self._state.mode != 'OFFBOARD':
            self._pub_and_spin()
            if self._elapsed(t0) > 5.0:
                log.warn("  OFFBOARD not confirmed, continuing...")
                break
        log.info(f"  Mode: {self._state.mode}")

        log.info("  Arming...")
        self.arm_vehicle(True)

        t0 = self.get_clock().now()
        while not self._state.armed:
            self._pub_and_spin()
            if self._elapsed(t0) > 10.0:
                log.error("  Failed to arm.")
                return
        log.info("  Armed.")

        # ── Step 3: Stable hover ─────────────────────────────
        log.info(f"\n[Step 3] Waiting for stable hover at {TARGET_ALT_M} m...")
        stable_since = None
        last_log     = self.get_clock().now()
        t_phase      = self.get_clock().now()

        while True:
            self._pub_and_spin()
            now     = self.get_clock().now()
            alt     = self.altitude()
            alt_err = TARGET_ALT_M - alt

            stable_since = (stable_since or now) if abs(alt_err) < ALT_TOL else None

            if (now - last_log).nanoseconds > 1e9:
                s = (now - stable_since).nanoseconds / 1e9 if stable_since else 0.0
                log.info(f"  alt={alt:.2f} m  stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
                last_log = now

            if stable_since and (now - stable_since).nanoseconds / 1e9 >= HOVER_STABLE_TIME:
                log.info(f"  Stable at {alt:.2f} m.")
                break
            if self._elapsed(t_phase) > 25.0:
                log.warn("  Timeout – continuing.")
                break

        # ── Step 4: Level hover hold ─────────────────────────
        log.info(f"\n[Step 4] Level hover hold for {HOVER_PHASE_DUR:.0f} s...")
        t0 = self.get_clock().now()
        while self._elapsed(t0) < HOVER_PHASE_DUR:
            self._pub_and_spin()

        x0, _ = self.position_xy()
        path_x0 = x0
        log.info(f"  Path start x = {path_x0:.2f} m (ENU East)")

        # ── Step 5: Forward + roll oscillation ───────────────
        log.info(f"\n[Step 5] Forward + roll ±{ROLL_AMP_DEG:.0f}°: "
                 f"{NUM_CYCLES} cycles × {ROLL_PERIOD_S:.0f} s = {TRACK_DUR_S:.0f} s")
        log.info(f"  {'Time':>6}  {'Xcmd':>6}  {'Xnow':>6}  {'Alt':>5}  "
                 f"{'RollCmd':>8}  {'RollNow':>8}")
        log.info("  " + "─" * 54)

        t_track    = self.get_clock().now()
        last_param = self.get_clock().now()
        last_log   = self.get_clock().now()
        roll_cmd   = 0.0

        while True:
            now     = self.get_clock().now()
            elapsed = (now - t_track).nanoseconds / 1e9
            if elapsed >= TRACK_DUR_S:
                elapsed = TRACK_DUR_S

            x_cmd    = path_x0 + elapsed * PATH_SPEED_MPS
            roll_cmd = ROLL_AMP_DEG * math.sin(OMEGA * elapsed)

            self._pub_and_spin(self._make_sp(x=x_cmd, y=0.0, z=TARGET_ALT_M, yaw=0.0))

            # param update at ~5 Hz (blocking; ROS 2 no background timer needed here
            # since spin_once() in _pub_and_spin() keeps callbacks alive)
            if (now - last_param).nanoseconds > 2e8:   # 0.2 s
                self.param_set('PFA_DES_ROLL', roll_cmd, retries=1)
                last_param = self.get_clock().now()

            if (now - last_log).nanoseconds > 5e8:
                xn, _ = self.position_xy()
                log.info(f"  {elapsed:6.1f}s  {x_cmd:6.2f}m  {xn:6.2f}m  "
                         f"{self.altitude():5.2f}m  {roll_cmd:8.1f}°  {self.roll_deg():8.1f}°")
                last_log = now

            if elapsed >= TRACK_DUR_S:
                break

        log.info("\n  Roll forward complete.")

        # ── Step 6: Reset + hold ─────────────────────────────
        log.info("\n[Step 6] Resetting PFA_DES_ROLL = 0°...")
        self.param_set('PFA_DES_ROLL', 0.0)

        xn, _ = self.position_xy()
        t0 = self.get_clock().now()
        while self._elapsed(t0) < 5.0:
            self._pub_and_spin(self._make_sp(x=xn, y=0.0))

        # ── Step 7: Land ─────────────────────────────────────
        log.info("\n[Step 7] Landing (AUTO.LAND)...")
        self.set_mode('AUTO.LAND')
        time.sleep(15.0)
        log.info("Done.")


# ─────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = PFARollForwardNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
