#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tianbot_core.msg import DmMitCommand, RoverMotionModeStatus
from tianbot_core.srv import SetControlMode

CMD_VEL_TOPIC = '/tianbot/cmd_vel'
WHEEL_MIT_TOPIC = '/tianbot/wheel_mit_cmd'
MODE_STATUS_TOPIC = '/tianbot/motion_mode_status'
SET_MODE_SERVICE = '/tianbot/set_control_mode'

DEFAULT_RATE_HZ = 100.0
DEFAULT_WZ = 2.0
DEFAULT_TFF_NM = 0.10
DEFAULT_WARMUP_ZERO_SEC = 0.5
DEFAULT_FORWARD_SEC = 4.0
DEFAULT_PAUSE_SEC = 2.0
DEFAULT_REVERSE_SEC = 4.0
DEFAULT_STOP_ZERO_SEC = 0.5
DEFAULT_WAIT_TIMEOUT_SEC = 15.0
DEFAULT_GRAPH_WARMUP_SEC = 2.5
DEFAULT_STATUS_WAIT_SEC = 3.0
DEFAULT_SET_MODE_RETRY = 2


class SpinInPlaceNode(Node):
    def __init__(self):
        super().__init__('spin_in_place_runner')
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.wheel_mit_pub = self.create_publisher(DmMitCommand, WHEEL_MIT_TOPIC, 10)
        self.mode_status = None
        self.mode_status_sub = self.create_subscription(
            RoverMotionModeStatus,
            MODE_STATUS_TOPIC,
            self._mode_status_callback,
            10,
        )
        self.set_mode_client = self.create_client(SetControlMode, SET_MODE_SERVICE)

    def _mode_status_callback(self, msg: RoverMotionModeStatus):
        self.mode_status = msg

    def warmup_graph(self, duration_sec: float):
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_service(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            if self.set_mode_client.wait_for_service(timeout_sec=0.2):
                return True
            rclpy.spin_once(self, timeout_sec=0.0)
        return False

    def wait_for_any_mode_status(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.mode_status is not None:
                return True
        return False

    def call_set_mode(self, mode: str, timeout_sec: float):
        request = SetControlMode.Request()
        request.mode = mode
        request.save_to_flash = False
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f'set_control_mode timeout for mode={mode}')
        response = future.result()
        if response is None:
            raise RuntimeError(f'set_control_mode returned empty response for mode={mode}')
        return response

    def is_mode_ready(self, expected_chassis_mode: str, expected_dm_mode: str) -> bool:
        msg = self.mode_status
        if msg is None:
            return False
        if msg.chassis_mode != expected_chassis_mode:
            return False
        if msg.dm_mode != expected_dm_mode:
            return False
        if not msg.ready:
            return False
        if any(mode != expected_dm_mode for mode in msg.motor_ctrl_mode):
            return False
        if any(state != 'online' for state in msg.motor_state):
            return False
        return True

    def wait_for_mode_ready(self, expected_chassis_mode: str, expected_dm_mode: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.is_mode_ready(expected_chassis_mode, expected_dm_mode):
                return True
        return False

    def ensure_mode_ready(self, mode: str, expected_chassis_mode: str, expected_dm_mode: str,
                          timeout_sec: float, status_wait_sec: float, retry_count: int):
        self.get_logger().info(f'Warmup ROS graph for {DEFAULT_GRAPH_WARMUP_SEC}s')
        self.warmup_graph(DEFAULT_GRAPH_WARMUP_SEC)

        self.get_logger().info(f'Waiting up to {status_wait_sec}s for current motion_mode_status')
        self.wait_for_any_mode_status(status_wait_sec)
        if self.is_mode_ready(expected_chassis_mode, expected_dm_mode):
            self.get_logger().info(f'Already in {expected_chassis_mode}/{expected_dm_mode}, skip set_control_mode')
            return

        last_error = None
        for attempt in range(1, retry_count + 1):
            self.get_logger().info(f'Switching to {mode} mode... attempt {attempt}/{retry_count}')
            try:
                response = self.call_set_mode(mode, timeout_sec)
                if not response.success:
                    last_error = response.message
                else:
                    self.get_logger().info(f'Waiting for motion_mode_status to confirm {mode} mode...')
                    if self.wait_for_mode_ready(expected_chassis_mode, expected_dm_mode, timeout_sec):
                        return
                    last_error = f'timed out waiting for {expected_chassis_mode}/{expected_dm_mode} ready state after service success'
            except Exception as exc:
                last_error = str(exc)

            self.get_logger().warn(f'Set mode attempt {attempt} failed: {last_error}')
            self.get_logger().info(f'Re-checking current motion_mode_status for {status_wait_sec}s before retry')
            self.wait_for_any_mode_status(status_wait_sec)
            if self.is_mode_ready(expected_chassis_mode, expected_dm_mode):
                self.get_logger().info(f'Now in {expected_chassis_mode}/{expected_dm_mode}, continue without retry')
                return

        raise RuntimeError(last_error or f'failed to switch to {mode}')

    def make_twist(self, wz: float) -> Twist:
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz
        return msg

    def make_mit(self, tff: float) -> DmMitCommand:
        msg = DmMitCommand()
        msg.p_des = [0.0, 0.0, 0.0, 0.0]
        msg.v_des = [0.0, 0.0, 0.0, 0.0]
        msg.kp = [0.0, 0.0, 0.0, 0.0]
        msg.kd = [0.0, 0.0, 0.0, 0.0]
        msg.t_ff = [tff, -tff, tff, -tff]
        return msg

    def publish_for(self, publisher, msg, duration_sec: float, rate_hz: float):
        period_sec = 1.0 / rate_hz
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)


def run_speed(node: SpinInPlaceNode, args):
    node.ensure_mode_ready('speed', 'pc_speed', 'speed', args.wait_timeout, args.status_wait_sec, args.set_mode_retry)
    node.get_logger().info(f'Warmup zero cmd_vel for {args.warmup_zero_sec}s')
    node.publish_for(node.cmd_vel_pub, node.make_twist(0.0), args.warmup_zero_sec, args.rate)
    node.get_logger().info(f'Forward spin for {args.forward_sec}s at wz={args.wz}')
    node.publish_for(node.cmd_vel_pub, node.make_twist(args.wz), args.forward_sec, args.rate)
    node.get_logger().info(f'Pause for {args.pause_sec}s')
    node.publish_for(node.cmd_vel_pub, node.make_twist(0.0), args.pause_sec, args.rate)
    node.get_logger().info(f'Reverse spin for {args.reverse_sec}s at wz={-args.wz}')
    node.publish_for(node.cmd_vel_pub, node.make_twist(-args.wz), args.reverse_sec, args.rate)
    node.get_logger().info(f'Stop with zero cmd_vel for {args.stop_zero_sec}s')
    node.publish_for(node.cmd_vel_pub, node.make_twist(0.0), args.stop_zero_sec, args.rate)


def run_mit(node: SpinInPlaceNode, args):
    node.ensure_mode_ready('mit', 'pc_mit', 'mit', args.wait_timeout, args.status_wait_sec, args.set_mode_retry)
    node.get_logger().info(f'Warmup zero torque for {args.warmup_zero_sec}s')
    node.publish_for(node.wheel_mit_pub, node.make_mit(0.0), args.warmup_zero_sec, args.rate)
    node.get_logger().info(f'Forward spin for {args.forward_sec}s at t_ff={args.tff}')
    node.publish_for(node.wheel_mit_pub, node.make_mit(args.tff), args.forward_sec, args.rate)
    node.get_logger().info(f'Pause for {args.pause_sec}s')
    node.publish_for(node.wheel_mit_pub, node.make_mit(0.0), args.pause_sec, args.rate)
    node.get_logger().info(f'Reverse spin for {args.reverse_sec}s at t_ff={-args.tff}')
    node.publish_for(node.wheel_mit_pub, node.make_mit(-args.tff), args.reverse_sec, args.rate)
    node.get_logger().info(f'Stop with zero torque for {args.stop_zero_sec}s')
    node.publish_for(node.wheel_mit_pub, node.make_mit(0.0), args.stop_zero_sec, args.rate)


def parse_args():
    parser = argparse.ArgumentParser(description='Spin in place using speed or MIT mode with persistent ROS2 publishing.')
    parser.add_argument('mode', choices=['speed', 'mit'])
    parser.add_argument('--wz', type=float, default=DEFAULT_WZ, help='Angular speed for speed mode, unit rad/s')
    parser.add_argument('--tff', type=float, default=DEFAULT_TFF_NM, help='Torque feed-forward for MIT mode, unit Nm')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ, help='Publish rate, unit Hz')
    parser.add_argument('--warmup-zero-sec', type=float, default=DEFAULT_WARMUP_ZERO_SEC)
    parser.add_argument('--forward-sec', type=float, default=DEFAULT_FORWARD_SEC)
    parser.add_argument('--pause-sec', type=float, default=DEFAULT_PAUSE_SEC)
    parser.add_argument('--reverse-sec', type=float, default=DEFAULT_REVERSE_SEC)
    parser.add_argument('--stop-zero-sec', type=float, default=DEFAULT_STOP_ZERO_SEC)
    parser.add_argument('--wait-timeout', type=float, default=DEFAULT_WAIT_TIMEOUT_SEC)
    parser.add_argument('--status-wait-sec', type=float, default=DEFAULT_STATUS_WAIT_SEC)
    parser.add_argument('--set-mode-retry', type=int, default=DEFAULT_SET_MODE_RETRY)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = SpinInPlaceNode()
    try:
        node.get_logger().info(
            f'Start spin test: mode={args.mode}, rate={args.rate} Hz, wz={args.wz}, tff={args.tff}'
        )
        if not node.wait_for_service(args.wait_timeout):
            raise RuntimeError(f'Service {SET_MODE_SERVICE} not available')
        if args.mode == 'speed':
            run_speed(node, args)
        else:
            run_mit(node, args)
        node.get_logger().info('Done.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
