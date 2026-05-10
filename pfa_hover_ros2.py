#!/usr/bin/env python3
"""
PFA Hover (MAVROS2 / ROS 2)
=============================
Task: 起飛懸停至高度 1 m，維持 30 秒，姿態全程 roll=pitch=0°

控制架構：
  Python → /mavros/setpoint_position/local (PoseStamped, ENU, z=+1 m)
         → orientation quaternion yaw=0° (防 NaN)
  PX4:  pfa_pos_control → thrust (維持 1 m 高度)
                        → attitude_des = (0, 0, 0)  ← PFA_DES_ROLL=PITCH=0
        pfa_att_control → roll=0°, pitch=0°

ROS 2 / MAVROS2 重點差異（vs ROS 1）：
  - rclpy.Node 類別架構
  - 非同步 service call：call_async + spin_until_future_complete
  - 時間：get_clock().now(), (t1-t0).nanoseconds / 1e9
  - spin：rclpy.spin_once(node, timeout_sec=0.05) 在迴圈中維持 callback

執行方式：
  python3 pfa_hover_ros2.py
"""

import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
TARGET_ALT_M      = 1.0    # 懸停高度 (m，ENU z+)
HOVER_DUR_S       = 30.0   # 維持時間 (s)
ALT_TOL           = 0.15   # 高度容忍 (m)
HOVER_STABLE_TIME = 3.0    # 穩定判定時間 (s)


# ─────────────────────────────────────────────────────────────
# Quaternion helper (避免 tf_transformations 依賴)
# ─────────────────────────────────────────────────────────────
def quat_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw*0.5),   math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5),  math.sin(roll*0.5)
    return (sr*cp*cy - cr*sp*sy,   # x
            cr*sp*cy + sr*cp*sy,   # y
            cr*cp*sy - sr*sp*cy,   # z
            cr*cp*cy + sr*sp*sy)   # w


def euler_from_quat(x, y, z, w):
    """Returns (roll, pitch, yaw) in radians."""
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sp    = 2*(w*y - z*x)
    pitch = math.asin(max(-1.0, min(1.0, sp)))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────
class PFAHoverNode(Node):

    def __init__(self):
        super().__init__('pfa_hover')

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscribers
        self.create_subscription(State,       '/mavros/state',               self._cb_state, reliable_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._cb_pose,  sensor_qos)
        self.create_subscription(Imu,         '/mavros/imu/data',            self._cb_imu,   sensor_qos)

        # Publisher
        self._sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', reliable_qos)

        # Service clients
        self._arm_cli   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_cli  = self.create_client(SetMode,     '/mavros/set_mode')
        self._param_cli = self.create_client(ParamSet,    '/mavros/param/set')

        # State
        self._state = State()
        self._pose  = PoseStamped()
        self._imu   = Imu()

    # ── Callbacks ────────────────────────────────────────────
    def _cb_state(self, msg): self._state = msg
    def _cb_pose(self, msg):  self._pose  = msg
    def _cb_imu(self, msg):   self._imu   = msg

    # ── Accessors ────────────────────────────────────────────
    def altitude(self):
        return self._pose.pose.position.z

    def roll_pitch_deg(self):
        o = self._imu.orientation
        r, p, _ = euler_from_quat(o.x, o.y, o.z, o.w)
        return math.degrees(r), math.degrees(p)

    # ── Setpoint ─────────────────────────────────────────────
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

    def _pub_sp(self):
        self._sp_pub.publish(self._make_sp())

    # ── Timing helpers ───────────────────────────────────────
    def _elapsed(self, t0):
        return (self.get_clock().now() - t0).nanoseconds / 1e9

    def _spin_for(self, secs):
        t0 = self.get_clock().now()
        while self._elapsed(t0) < secs:
            self._pub_sp()
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── Services ─────────────────────────────────────────────
    def param_set(self, name, value, retries=3):
        if not self._param_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'param/set service unavailable')
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
        log.info(f"Task: hover {TARGET_ALT_M} m for {HOVER_DUR_S:.0f} s, attitude=0°")

        # Wait for FCU connection
        log.info("Waiting for FCU connection...")
        while not self._state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
        log.info(f"  Connected. mode={self._state.mode}")

        # ── Step 0: PFA params ──────────────────────────────
        log.info("\n[Step 0] Setting PFA_DES_ROLL/PITCH = 0°...")
        log.info("  PFA_DES_ROLL  = 0° ... " + ("OK" if self.param_set('PFA_DES_ROLL',  0.0) else "FAILED"))
        log.info("  PFA_DES_PITCH = 0° ... " + ("OK" if self.param_set('PFA_DES_PITCH', 0.0) else "FAILED"))

        # ── Step 1: Stream setpoints (OFFBOARD precondition) ─
        log.info(f"\n[Step 1] Streaming setpoints (5 s, z={TARGET_ALT_M} m)...")
        self._spin_for(5.0)

        # ── Step 2: OFFBOARD + Arm ──────────────────────────
        log.info("  Requesting OFFBOARD mode...")
        self.set_mode('OFFBOARD')

        t0 = self.get_clock().now()
        while self._state.mode != 'OFFBOARD':
            self._pub_sp()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._elapsed(t0) > 5.0:
                log.warn("  OFFBOARD not confirmed, continuing...")
                break
        log.info(f"  Mode: {self._state.mode}")

        log.info("  Arming...")
        self.arm_vehicle(True)

        t0 = self.get_clock().now()
        while not self._state.armed:
            self._pub_sp()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._elapsed(t0) > 10.0:
                log.error("  Failed to arm.")
                return
        log.info("  Armed.")

        # ── Step 3: Stable hover ────────────────────────────
        log.info(f"\n[Step 3] Waiting for stable hover (±{ALT_TOL} m for {HOVER_STABLE_TIME:.0f} s)...")

        stable_since = None
        last_log     = self.get_clock().now()
        t_phase      = self.get_clock().now()

        while True:
            self._pub_sp()
            rclpy.spin_once(self, timeout_sec=0.05)

            now     = self.get_clock().now()
            alt     = self.altitude()
            alt_err = TARGET_ALT_M - alt

            if abs(alt_err) < ALT_TOL:
                if stable_since is None:
                    stable_since = now
            else:
                stable_since = None

            if (now - last_log).nanoseconds > 1e9:
                s = (now - stable_since).nanoseconds / 1e9 if stable_since else 0.0
                log.info(f"  alt={alt:.2f} m (err={alt_err:+.2f})  stable={s:.1f}/{HOVER_STABLE_TIME:.0f} s")
                last_log = now

            if stable_since and (now - stable_since).nanoseconds / 1e9 >= HOVER_STABLE_TIME:
                log.info(f"  Stable at {alt:.2f} m.")
                break
            if self._elapsed(t_phase) > 25.0:
                log.warn(f"  Timeout – alt={alt:.2f} m, continuing.")
                break

        # ── Step 4: 30 s hover hold ─────────────────────────
        log.info(f"\n[Step 4] Hovering for {HOVER_DUR_S:.0f} s (roll=pitch=0°)...")
        log.info(f"  {'Time':>6}  {'Alt':>6}  {'AltErr':>7}  {'Roll':>7}  {'Pitch':>7}")
        log.info("  " + "─" * 44)

        t_hold   = self.get_clock().now()
        last_log = self.get_clock().now()

        while True:
            self._pub_sp()
            rclpy.spin_once(self, timeout_sec=0.05)

            now     = self.get_clock().now()
            elapsed = (now - t_hold).nanoseconds / 1e9

            if (now - last_log).nanoseconds > 5e8:   # 0.5 s
                alt          = self.altitude()
                r_deg, p_deg = self.roll_pitch_deg()
                log.info(f"  {elapsed:6.1f}s  {alt:6.2f}m  {TARGET_ALT_M-alt:+7.2f}m  "
                         f"{r_deg:6.1f}°  {p_deg:6.1f}°")
                last_log = now

            if elapsed >= HOVER_DUR_S:
                break

        log.info("  Hover complete.")

        # ── Step 5: Land ────────────────────────────────────
        log.info("\n[Step 5] Landing (AUTO.LAND)...")
        self.set_mode('AUTO.LAND')
        time.sleep(15.0)
        log.info("Done.")


# ─────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = PFAHoverNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
