#!/usr/bin/env python3
"""
Trajectory Test via MAVROS (ROS2)
=================================
透過 MAVROS 發送位置 setpoint 執行預定義軌跡。
採用與 pfa_attitude_tracking_mavros.py 相同的控制架構，
確保 offboard 訊號持續不中斷。

控制架構：
  Python → /mavros/setpoint_position/local (ENU, PoseStamped)
                    ↓ MAVROS 自動 ENU→NED 轉換
  PX4：  SET_POSITION_TARGET_LOCAL_NED (MAVLink)
                    ↓ mavlink_receiver
         trajectory_setpoint + offboard_control_mode (uORB)
                    ↓
         FlightTaskOffboard → mc_pos_control

座標系：ENU (x=East, y=North, z=Up, yaw=0 朝 East, 逆時針為正)

Usage:
  python3 trajectory_test_mavros.py hover
  python3 trajectory_test_mavros.py square
  python3 trajectory_test_mavros.py circle
"""

import rclpy
from rclpy.node import Node
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────
CTRL_HZ          = 20        # 控制迴路頻率 (Hz)
POS_THRESHOLD    = 0.3       # 到達 waypoint 判定距離 (m)
WP_TIMEOUT       = 30.0      # 單一 waypoint 超時 (s)
PRE_OFFBOARD_SEC = 5.0       # 進入 OFFBOARD 前預送 setpoint 時間 (s)

# ─────────────────────────────────────────────────────────────
# Quaternion helpers (避免額外 dependency)
# ─────────────────────────────────────────────────────────────
def quaternion_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def euler_from_quaternion(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────
# Trajectory Definitions (ENU frame)
# ─────────────────────────────────────────────────────────────
# Waypoint: (x, y, z, yaw, hold_time)
#   x=East(m), y=North(m), z=Up(m)
#   yaw: rad, 0=East, pi/2=North; None=維持當前 yaw
#   hold_time: 到達後停留秒數

def get_trajectory(name):
    PI = math.pi
    H = 1.0
    HOLD = 2.0

    if name == 'hover':
        return {
            'name': 'hover',
            'waypoints': [
                (0.0, 0.0, 0.5, None, 10.0),
            ],
            'threshold': 0.2,
        }

    elif name == 'square':
        return {
            'name': 'square',
            'waypoints': [
                (0.0, 0.0, H,  PI / 2,  HOLD),   # takeoff, face North
                (0.0, 1.0, H,  PI / 2,  HOLD),   # fly North
                (0.0, 1.0, H,  0.0,     HOLD),   # rotate face East
                (1.0, 1.0, H,  0.0,     HOLD),   # fly East
                (1.0, 1.0, H, -PI / 2,  HOLD),   # rotate face South
                (1.0, 0.0, H, -PI / 2,  HOLD),   # fly South
                (1.0, 0.0, H,  PI,      HOLD),   # rotate face West
                (0.0, 0.0, H,  PI,      HOLD),   # fly West (back)
                (0.0, 0.0, H,  PI / 2,  HOLD),   # rotate back to North
            ],
            'threshold': 0.3,
        }

    elif name == 'circle':
        N = 16
        R = 1.0
        CX, CY = 1.0, 0.0
        wps = []
        for i in range(N):
            angle = PI + (2.0 * PI * i) / N
            x = CX + R * math.cos(angle)
            y = CY + R * math.sin(angle)
            yaw = angle + PI / 2.0
            while yaw > PI:
                yaw -= 2.0 * PI
            while yaw < -PI:
                yaw += 2.0 * PI
            wps.append((x, y, H, yaw, 1.0))
        wps.append((0.0, 0.0, H, PI / 2, HOLD))
        return {
            'name': 'circle',
            'waypoints': wps,
            'threshold': 0.3,
        }

    return None


# ─────────────────────────────────────────────────────────────
# Global state (subscriber callbacks)
# ─────────────────────────────────────────────────────────────
_vehicle_state = State()
_local_pose = PoseStamped()


def _cb_state(msg):
    global _vehicle_state
    _vehicle_state = msg


def _cb_pose(msg):
    global _local_pose
    _local_pose = msg


def get_position():
    p = _local_pose.pose.position
    return p.x, p.y, p.z


def get_yaw():
    o = _local_pose.pose.orientation
    _, _, yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)
    return yaw


def distance_to(x, y, z):
    px, py, pz = get_position()
    return math.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2)


# ─────────────────────────────────────────────────────────────
# Setpoint helper
# ─────────────────────────────────────────────────────────────
def make_setpoint(node, x, y, z, yaw=None):
    """
    建立 ENU PoseStamped setpoint。
    MAVROS setpoint_position plugin 將自動轉換為
    SET_POSITION_TARGET_LOCAL_NED (type_mask=position+yaw)，
    確保 trajectory_setpoint.yaw 為有限值。
    """
    if yaw is None:
        yaw = get_yaw()

    sp = PoseStamped()
    sp.header.stamp = node.get_clock().now().to_msg()
    sp.header.frame_id = 'map'
    sp.pose.position.x = float(x)
    sp.pose.position.y = float(y)
    sp.pose.position.z = float(z)

    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    sp.pose.orientation.x = qx
    sp.pose.orientation.y = qy
    sp.pose.orientation.z = qz
    sp.pose.orientation.w = qw
    return sp


# ─────────────────────────────────────────────────────────────
# Service wrappers (同 pfa_attitude_tracking_mavros.py 風格)
# ─────────────────────────────────────────────────────────────
def set_mode(node, client, mode_str, retries=5):
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('set_mode service not available')
        return False
    for _ in range(retries):
        req = SetMode.Request()
        req.custom_mode = mode_str
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is not None and future.result().mode_sent:
            return True
        time.sleep(0.3)
    return False


def arm_vehicle(node, client, do_arm=True, retries=5):
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('arming service not available')
        return False
    for _ in range(retries):
        req = CommandBool.Request()
        req.value = do_arm
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is not None and future.result().success:
            return True
        time.sleep(0.3)
    return False


# ─────────────────────────────────────────────────────────────
# Publish setpoint + spin once (保持 offboard 不中斷)
# ─────────────────────────────────────────────────────────────
def tick(node, sp_pub, x, y, z, yaw=None):
    rclpy.spin_once(node, timeout_sec=0)
    sp_pub.publish(make_setpoint(node, x, y, z, yaw))
    time.sleep(1.0 / CTRL_HZ)


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main():
    traj_name = sys.argv[1] if len(sys.argv) > 1 else 'hover'
    traj = get_trajectory(traj_name)
    if traj is None:
        print(f"Unknown trajectory: '{traj_name}'. Available: hover, square, circle")
        sys.exit(1)

    rclpy.init()
    node = Node('trajectory_test')
    log = node.get_logger()

    # Subscribers
    node.create_subscription(State, '/mavros/state', _cb_state, 10)
    node.create_subscription(PoseStamped, '/mavros/local_position/pose', _cb_pose, 10)

    # Publisher
    sp_pub = node.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

    # Service clients
    arm_client = node.create_client(CommandBool, '/mavros/cmd/arming')
    mode_client = node.create_client(SetMode, '/mavros/set_mode')

    waypoints = traj['waypoints']
    threshold = traj['threshold']
    num_wps = len(waypoints)
    wp0 = waypoints[0]

    log.info(f'Trajectory: {traj["name"]} ({num_wps} waypoints)')

    # ── Wait for MAVROS FCU connection ──
    log.info('Waiting for MAVROS FCU connection...')
    while rclpy.ok() and not _vehicle_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)
    log.info(f'Connected. Mode: {_vehicle_state.mode}')

    # ── Step 1: Stream initial setpoints ──
    log.info(f'[Step 1] Streaming setpoints ({PRE_OFFBOARD_SEC:.0f}s)...')
    t0 = time.time()
    while rclpy.ok() and (time.time() - t0) < PRE_OFFBOARD_SEC:
        tick(node, sp_pub, wp0[0], wp0[1], wp0[2], wp0[3])

    # ── Step 2: OFFBOARD + Arm ──
    log.info('[Step 2] Requesting OFFBOARD mode...')
    set_mode(node, mode_client, 'OFFBOARD')

    t_wait = time.time()
    while rclpy.ok() and _vehicle_state.mode != 'OFFBOARD':
        tick(node, sp_pub, wp0[0], wp0[1], wp0[2], wp0[3])
        if time.time() - t_wait > 10.0:
            log.warn('OFFBOARD not confirmed after 10s, continuing...')
            break

    log.info(f'Mode: {_vehicle_state.mode}')

    log.info('Arming...')
    arm_vehicle(node, arm_client)

    t_wait = time.time()
    while rclpy.ok() and not _vehicle_state.armed:
        tick(node, sp_pub, wp0[0], wp0[1], wp0[2], wp0[3])
        if time.time() - t_wait > 10.0:
            log.error('Arm timeout. Aborting.')
            node.destroy_node()
            rclpy.shutdown()
            return

    log.info('Armed!')

    # ── Step 3: Fly waypoints ──
    log.info(f'[Step 3] Flying {traj["name"]} trajectory...')

    for wp_idx in range(num_wps):
        x, y, z, yaw, hold_time = waypoints[wp_idx]
        yaw_str = f'{math.degrees(yaw):.0f}' if yaw is not None else 'hold'
        log.info(f'[WP {wp_idx + 1}/{num_wps}] -> ({x:.2f}, {y:.2f}, {z:.2f}) yaw={yaw_str} deg')

        # Fly to waypoint
        t_wp = time.time()
        t_last_log = time.time()
        while rclpy.ok():
            tick(node, sp_pub, x, y, z, yaw)
            dist = distance_to(x, y, z)

            now = time.time()
            if now - t_last_log > 2.0:
                px, py, pz = get_position()
                log.info(f'  pos=({px:.2f}, {py:.2f}, {pz:.2f}) dist={dist:.2f}m')
                t_last_log = now

            if dist < threshold:
                log.info(f'  Reached WP {wp_idx + 1} (dist={dist:.2f}m)')
                break

            if now - t_wp > WP_TIMEOUT:
                log.warn(f'  WP {wp_idx + 1} timeout (dist={dist:.2f}m). Continuing.')
                break

        # Hold at waypoint
        log.info(f'  Holding {hold_time:.1f}s...')
        t_hold = time.time()
        while rclpy.ok() and (time.time() - t_hold) < hold_time:
            tick(node, sp_pub, x, y, z, yaw)

    log.info(f'{traj["name"]} trajectory complete!')

    # ── Step 4: Land ──
    log.info('[Step 4] Landing (AUTO.LAND)...')
    set_mode(node, mode_client, 'AUTO.LAND')

    t_land = time.time()
    while rclpy.ok() and (time.time() - t_land) < 30.0:
        rclpy.spin_once(node, timeout_sec=0)
        if not _vehicle_state.armed:
            log.info('Disarmed. Landing complete.')
            break
        now = time.time()
        if int(now - t_land) % 5 == 0 and (now - t_land) > 1.0:
            px, py, pz = get_position()
            log.info(f'  Landing... alt={pz:.2f}m')
        time.sleep(1.0 / CTRL_HZ)

    log.info('Done.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrupted.')
        rclpy.try_shutdown()
