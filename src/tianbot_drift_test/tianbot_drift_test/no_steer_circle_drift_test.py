"""ROS 2 test node for the Tianbot DYN no-steer drift car.

The controller uses IMU yaw rate as the primary feedback signal. Odometry is
used only for a weak speed loop and, when explicitly enabled, a weak radius
outer loop. Controller wheel order is always FL, FR, RL, RR. The DYN MIT
interface already normalizes the installed motor directions, so the default
torque signs are all positive; per-wheel overrides remain available for other
hardware configurations.
"""

import csv
import math
import os
import threading
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from tianbot_core.msg import DmMitCommand, RoverMotionModeStatus


class NoSteerCircleDriftTest(Node):
    PHASE_IDLE = 0
    PHASE_ACCEL = 1
    PHASE_INITIATE = 2
    PHASE_HOLD = 3
    PHASE_STOP = 4
    PHASE_ABORT = 5

    PHASE_NAMES = {
        PHASE_IDLE: "idle",
        PHASE_ACCEL: "accelerate",
        PHASE_INITIATE: "initiate",
        PHASE_HOLD: "hold",
        PHASE_STOP: "stop",
        PHASE_ABORT: "abort",
    }

    def __init__(self):
        super().__init__("no_steer_circle_drift_test")
        self.lock = threading.Lock()

        self.rate_hz = float(self.param("control_rate_hz", 100.0))
        self.imu_topic = self.param("imu_topic", "/tianbot/imu")
        self.odom_topic = self.param("odom_topic", "/tianbot/odom")
        self.cmd_topic = self.param(
            "wheel_torque_command_topic", "/tianbot/wheel_mit_cmd"
        )
        self.diag_topic = self.param(
            "diagnostics_topic", "/drift_test/diagnostics"
        )
        self.mode_status_topic = self.param(
            "mode_status_topic", "/tianbot/motion_mode_status"
        )
        self.require_mit_mode = bool(self.param("require_mit_mode", True))
        self.motor_torque_signs = [
            float(value)
            for value in self.param(
                "motor_torque_signs", [1.0, 1.0, 1.0, 1.0]
            )
        ]
        if len(self.motor_torque_signs) != 4:
            raise ValueError("motor_torque_signs must contain FL, FR, RL, RR")

        self.direction = 1.0 if float(self.param("direction", 1)) >= 0 else -1.0
        self.target_radius = float(self.param("target_radius_m", 0.60))
        self.target_speed = float(self.param("target_speed_mps", 1.00))
        self.hold_laps = float(self.param("hold_laps", 3.0))

        self.accel_min_time = float(self.param("accel_min_time_s", 0.35))
        self.accel_timeout = float(self.param("accel_timeout_s", 1.20))
        self.accel_torque = float(self.param("accel_torque_nm", 0.075))
        self.accel_exit_speed_ratio = float(
            self.param("accel_exit_speed_ratio", 0.90)
        )
        self.initiate_duration = float(self.param("initiate_duration_s", 0.30))
        self.initiate_base_torque = float(
            self.param("initiate_base_torque_nm", 0.070)
        )
        self.initiate_diff_torque = float(
            self.param("initiate_diff_torque_nm", 0.040)
        )

        self.hold_base_torque = float(self.param("hold_base_torque_nm", 0.040))
        self.yaw_kp = float(self.param("yaw_rate_kp_nm_per_radps", 0.018))
        self.yaw_ki = float(self.param("yaw_rate_ki_nm_per_rad", 0.006))
        self.yaw_integral_limit = float(
            self.param("yaw_integral_limit_rad", 1.5)
        )
        self.max_yaw_diff_torque = float(
            self.param("max_yaw_diff_torque_nm", 0.030)
        )

        self.use_odom_speed = bool(self.param("use_odom_speed_feedback", True))
        self.speed_kp = float(self.param("speed_kp_nm_per_mps", 0.020))
        self.max_speed_correction = float(
            self.param("max_speed_correction_nm", 0.020)
        )

        self.use_odom_radius = bool(self.param("use_odom_radius_feedback", False))
        self.radius_kp = float(self.param("radius_to_yaw_gain_radps_per_m", 0.35))
        self.radius_kd = float(
            self.param("radial_velocity_to_yaw_gain_rad_per_m", 0.08)
        )
        self.radius_feedback_delay = float(
            self.param("radius_feedback_delay_s", 3.8)
        )
        self.max_radius_yaw_bias = float(
            self.param("max_radius_yaw_bias_radps", 0.20)
        )
        self.radius_filter_tau = float(self.param("radius_filter_tau_s", 0.25))

        self.max_wheel_torque = float(self.param("max_wheel_torque_nm", 0.12))
        self.max_torque_slew = float(self.param("max_torque_slew_nmps", 0.60))
        self.stop_duration = float(self.param("stop_duration_s", 0.60))
        self.sensor_timeout = float(self.param("sensor_timeout_s", 0.20))
        self.max_abs_yaw_rate = float(self.param("max_abs_yaw_rate_radps", 4.0))
        self.max_speed = float(self.param("max_speed_mps", 2.5))
        self.gyro_bias_min_samples = int(self.param("gyro_bias_min_samples", 80))
        self.log_directory = os.path.expanduser(
            self.param("log_directory", "~/.ros/drift_test_logs")
        )

        self.phase = self.PHASE_IDLE
        self.phase_start = self.get_clock().now()
        self.last_control_time = self.phase_start
        self.test_start = None
        self.abort_reason = ""
        self.last_imu_stamp = None
        self.last_odom_stamp = None
        self.mode_status_received = False
        self.mit_mode_ready = False
        self.yaw_rate_raw = 0.0
        self.yaw_rate = 0.0
        self.speed = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.gyro_idle_samples = deque(maxlen=500)
        self.gyro_bias = 0.0
        self.yaw_integral = 0.0
        self.circle_center = None
        self.radius_raw = float("nan")
        self.radius_filtered = float("nan")
        self.radius_rate = 0.0
        self.previous_radius_filtered = float("nan")
        self.previous_command = [0.0, 0.0, 0.0, 0.0]
        self.stop_start_command = [0.0, 0.0, 0.0, 0.0]
        self.log_file = None
        self.log_writer = None

        self.cmd_pub = self.create_publisher(DmMitCommand, self.cmd_topic, 1)
        self.diag_pub = self.create_publisher(Float64MultiArray, self.diag_topic, 10)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.mode_status_sub = self.create_subscription(
            RoverMotionModeStatus,
            self.mode_status_topic,
            self.mode_status_callback,
            10,
        )
        self.start_service = self.create_service(Trigger, "~/start", self.start_callback)
        self.abort_service = self.create_service(Trigger, "~/abort", self.abort_callback)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_callback)

        self.get_logger().info(
            "No-steer drift test ready: R=%.2f m, V=%.2f m/s, r_ref=%.3f rad/s, %s"
            % (
                self.target_radius,
                self.target_speed,
                self.nominal_yaw_rate,
                "CCW" if self.direction > 0 else "CW",
            )
        )
        self.get_logger().warning(
            "Wheel order is [FL, FR, RL, RR], motor signs are %s. "
            "Verify signs with wheels off the ground."
            % self.motor_torque_signs
        )

    def param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    @property
    def nominal_yaw_rate(self):
        return self.direction * self.target_speed / self.target_radius

    @property
    def hold_duration(self):
        return self.hold_laps * 2.0 * math.pi * self.target_radius / self.target_speed

    def imu_callback(self, msg):
        receive_time = self.get_clock().now()
        with self.lock:
            self.last_imu_stamp = receive_time
            self.yaw_rate_raw = msg.angular_velocity.z
            self.yaw_rate = self.yaw_rate_raw - self.gyro_bias
            if self.phase == self.PHASE_IDLE and abs(self.speed) < 0.10:
                if math.isfinite(self.yaw_rate_raw) and abs(self.yaw_rate_raw) < 0.30:
                    self.gyro_idle_samples.append(self.yaw_rate_raw)
                    self.gyro_bias = sum(self.gyro_idle_samples) / len(self.gyro_idle_samples)

    def odom_callback(self, msg):
        receive_time = self.get_clock().now()
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        with self.lock:
            self.last_odom_stamp = receive_time
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            self.odom_yaw = yaw
            self.speed = speed

    def mode_status_callback(self, msg):
        ready = (
            msg.chassis_mode == "pc_mit"
            and msg.dm_mode == "mit"
            and msg.ready
            and all(mode == "mit" for mode in msg.motor_ctrl_mode)
            and all(state == "online" for state in msg.motor_state)
        )
        with self.lock:
            self.mode_status_received = True
            self.mit_mode_ready = ready

    def start_callback(self, _request, response):
        now = self.get_clock().now()
        with self.lock:
            if self.phase != self.PHASE_IDLE:
                response.success = False
                response.message = "test is already active"
                return response
            ok, reason = self.sensors_ready(now)
            if not ok:
                response.success = False
                response.message = reason
                return response
            if len(self.gyro_idle_samples) < self.gyro_bias_min_samples:
                response.success = False
                response.message = (
                    "keep car stationary for IMU bias calibration (%d/%d samples)"
                    % (len(self.gyro_idle_samples), self.gyro_bias_min_samples)
                )
                return response
            self.gyro_bias = sum(self.gyro_idle_samples) / len(self.gyro_idle_samples)
            self.test_start = now
            self.phase_start = now
            self.phase = self.PHASE_ACCEL
            self.abort_reason = ""
            self.yaw_integral = 0.0
            self.circle_center = None
            self.radius_raw = float("nan")
            self.radius_filtered = float("nan")
            self.previous_radius_filtered = float("nan")
            self.previous_command = [0.0, 0.0, 0.0, 0.0]
            self.open_log(now)
            self.get_logger().warning("Drift test STARTED. Clear the test area.")
            response.success = True
            response.message = "drift test started"
            return response

    def abort_callback(self, _request, response):
        with self.lock:
            if self.phase == self.PHASE_IDLE:
                self.publish_command([0.0, 0.0, 0.0, 0.0])
                response.success = True
                response.message = "already idle; zero command published"
                return response
            self.enter_abort("manual abort")
            response.success = True
            response.message = "abort requested"
            return response

    def sensors_ready(self, now):
        if self.require_mit_mode and not self.mode_status_received:
            return False, "no motion-mode status received"
        if self.require_mit_mode and not self.mit_mode_ready:
            return False, "DYN chassis is not ready in PC_MIT/mit mode"
        if self.last_imu_stamp is None:
            return False, "no IMU message received"
        if self.last_odom_stamp is None:
            return False, "no odometry message received"
        if self.elapsed(now, self.last_imu_stamp) > self.sensor_timeout:
            return False, "IMU data is stale"
        if self.elapsed(now, self.last_odom_stamp) > self.sensor_timeout:
            return False, "odometry data is stale"
        return True, ""

    def control_callback(self):
        now = self.get_clock().now()
        dt = max(min(self.elapsed(now, self.last_control_time), 0.05), 1e-4)
        self.last_control_time = now
        with self.lock:
            if self.phase == self.PHASE_IDLE:
                self.publish_command([0.0, 0.0, 0.0, 0.0])
                self.publish_diagnostics(now, self.nominal_yaw_rate, 0.0)
                return

            ok, reason = self.sensors_ready(now)
            if not ok:
                self.enter_abort(reason)
            elif not math.isfinite(self.yaw_rate) or not math.isfinite(self.speed):
                self.enter_abort("non-finite sensor value")
            elif abs(self.yaw_rate) > self.max_abs_yaw_rate:
                self.enter_abort("yaw-rate safety limit exceeded")
            elif self.speed > self.max_speed:
                self.enter_abort("speed safety limit exceeded")

            phase_time = self.elapsed(now, self.phase_start)
            raw_command = [0.0, 0.0, 0.0, 0.0]
            yaw_ref = self.nominal_yaw_rate
            radius_bias = 0.0

            if self.phase == self.PHASE_ACCEL:
                ramp = min(phase_time / max(self.accel_min_time, 0.05), 1.0)
                raw_command = [self.accel_torque * ramp] * 4
                reached_speed = self.speed >= self.accel_exit_speed_ratio * self.target_speed
                if (phase_time >= self.accel_min_time and reached_speed) or phase_time >= self.accel_timeout:
                    self.enter_phase(self.PHASE_INITIATE, now)
                    self.set_circle_center_from_odom()

            elif self.phase == self.PHASE_INITIATE:
                left = self.initiate_base_torque - self.direction * self.initiate_diff_torque
                right = self.initiate_base_torque + self.direction * self.initiate_diff_torque
                raw_command = [left, right, left, right]
                if phase_time >= self.initiate_duration:
                    self.enter_phase(self.PHASE_HOLD, now)
                    self.yaw_integral = 0.0

            elif self.phase == self.PHASE_HOLD:
                self.update_radius_estimate(dt)
                radius_bias = self.calculate_radius_yaw_bias(phase_time)
                yaw_ref = self.nominal_yaw_rate + radius_bias
                yaw_error = yaw_ref - self.yaw_rate
                self.yaw_integral += yaw_error * dt
                self.yaw_integral = self.clamp(
                    self.yaw_integral,
                    -self.yaw_integral_limit,
                    self.yaw_integral_limit,
                )
                diff = self.yaw_kp * yaw_error + self.yaw_ki * self.yaw_integral
                diff = self.clamp(
                    diff, -self.max_yaw_diff_torque, self.max_yaw_diff_torque
                )
                speed_correction = 0.0
                if self.use_odom_speed:
                    speed_correction = self.clamp(
                        self.speed_kp * (self.target_speed - self.speed),
                        -self.max_speed_correction,
                        self.max_speed_correction,
                    )
                base = self.hold_base_torque + speed_correction
                # Positive signed yaw error always requires more right-side torque.
                left = base - diff
                right = base + diff
                raw_command = [left, right, left, right]
                if phase_time >= self.hold_duration:
                    self.stop_start_command = list(self.previous_command)
                    self.enter_phase(self.PHASE_STOP, now)

            elif self.phase == self.PHASE_STOP:
                scale = max(0.0, 1.0 - phase_time / max(self.stop_duration, 0.05))
                raw_command = [value * scale for value in self.stop_start_command]
                if phase_time >= self.stop_duration:
                    self.finish_test("completed")
                    return

            elif self.phase == self.PHASE_ABORT:
                raw_command = [0.0, 0.0, 0.0, 0.0]
                if phase_time >= 0.30:
                    self.finish_test("aborted: " + self.abort_reason)
                    return

            command = self.limit_and_slew(raw_command, dt)
            self.publish_command(command)
            self.publish_diagnostics(now, yaw_ref, radius_bias)
            self.write_log(now, yaw_ref, radius_bias, command)

    def enter_phase(self, phase, now):
        self.phase = phase
        self.phase_start = now
        self.get_logger().info("Drift test phase: %s" % self.PHASE_NAMES[phase])

    def enter_abort(self, reason):
        if self.phase != self.PHASE_ABORT:
            self.phase = self.PHASE_ABORT
            self.phase_start = self.get_clock().now()
            self.abort_reason = reason
            self.previous_command = [0.0, 0.0, 0.0, 0.0]
            self.publish_command(self.previous_command)
            self.get_logger().error("Drift test ABORT: %s" % reason)

    def finish_test(self, result):
        self.publish_command([0.0, 0.0, 0.0, 0.0])
        self.previous_command = [0.0, 0.0, 0.0, 0.0]
        self.phase = self.PHASE_IDLE
        self.phase_start = self.get_clock().now()
        self.close_log()
        self.get_logger().warning("Drift test finished: %s" % result)

    def set_circle_center_from_odom(self):
        # Before initiation, body heading approximates the path tangent.
        nx = -math.sin(self.odom_yaw)
        ny = math.cos(self.odom_yaw)
        self.circle_center = (
            self.odom_x + self.direction * self.target_radius * nx,
            self.odom_y + self.direction * self.target_radius * ny,
        )
        self.get_logger().info(
            "Odom circle center initialized at (%.3f, %.3f)"
            % (self.circle_center[0], self.circle_center[1])
        )

    def update_radius_estimate(self, dt):
        if self.circle_center is None:
            return
        dx = self.odom_x - self.circle_center[0]
        dy = self.odom_y - self.circle_center[1]
        self.radius_raw = math.hypot(dx, dy)
        if not math.isfinite(self.radius_filtered):
            self.radius_filtered = self.radius_raw
            self.previous_radius_filtered = self.radius_raw
            self.radius_rate = 0.0
            return
        alpha = dt / (self.radius_filter_tau + dt)
        self.radius_filtered += alpha * (self.radius_raw - self.radius_filtered)
        raw_rate = (self.radius_filtered - self.previous_radius_filtered) / dt
        self.radius_rate += alpha * (raw_rate - self.radius_rate)
        self.previous_radius_filtered = self.radius_filtered

    def calculate_radius_yaw_bias(self, hold_time):
        if not self.use_odom_radius or hold_time < self.radius_feedback_delay:
            return 0.0
        if not math.isfinite(self.radius_filtered):
            return 0.0
        error = self.radius_filtered - self.target_radius
        magnitude_bias = self.radius_kp * error + self.radius_kd * self.radius_rate
        magnitude_bias = self.clamp(
            magnitude_bias, -self.max_radius_yaw_bias, self.max_radius_yaw_bias
        )
        return self.direction * magnitude_bias

    def limit_and_slew(self, raw, dt):
        limited = [self.clamp(x, -self.max_wheel_torque, self.max_wheel_torque) for x in raw]
        max_step = self.max_torque_slew * dt
        output = [
            previous + self.clamp(target - previous, -max_step, max_step)
            for target, previous in zip(limited, self.previous_command)
        ]
        self.previous_command = output
        return output

    def publish_command(self, command):
        msg = DmMitCommand()
        msg.p_des = [0.0, 0.0, 0.0, 0.0]
        msg.v_des = [0.0, 0.0, 0.0, 0.0]
        msg.kp = [0.0, 0.0, 0.0, 0.0]
        msg.kd = [0.0, 0.0, 0.0, 0.0]
        msg.t_ff = [
            float(torque * sign)
            for torque, sign in zip(command, self.motor_torque_signs)
        ]
        self.cmd_pub.publish(msg)

    def publish_diagnostics(self, now, yaw_ref, radius_bias):
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(
                label=(
                    "phase,yaw_rate,yaw_ref,speed,radius_raw,radius_filtered,"
                    "radius_rate,radius_yaw_bias,gyro_bias,odom_x,odom_y"
                ),
                size=11,
                stride=11,
            )
        ]
        msg.data = [
            float(self.phase),
            self.yaw_rate,
            yaw_ref,
            self.speed,
            self.radius_raw,
            self.radius_filtered,
            self.radius_rate,
            radius_bias,
            self.gyro_bias,
            self.odom_x,
            self.odom_y,
        ]
        self.diag_pub.publish(msg)

    def open_log(self, now):
        os.makedirs(self.log_directory, exist_ok=True)
        stamp = now.nanoseconds * 1e-9
        path = os.path.join(self.log_directory, "drift_test_%.3f.csv" % stamp)
        self.log_file = open(path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(
            [
                "time_s", "phase", "yaw_rate_radps", "yaw_ref_radps",
                "speed_odom_mps", "odom_x_m", "odom_y_m", "odom_yaw_rad",
                "radius_raw_m", "radius_filtered_m", "radius_rate_mps",
                "radius_yaw_bias_radps", "gyro_bias_radps",
                "torque_FL_Nm", "torque_FR_Nm", "torque_RL_Nm", "torque_RR_Nm",
            ]
        )
        self.get_logger().info("Logging drift test to %s" % path)

    def write_log(self, now, yaw_ref, radius_bias, command):
        if self.log_writer is None or self.test_start is None:
            return
        self.log_writer.writerow(
            [
                self.elapsed(now, self.test_start), self.PHASE_NAMES[self.phase],
                self.yaw_rate, yaw_ref, self.speed, self.odom_x, self.odom_y,
                self.odom_yaw, self.radius_raw, self.radius_filtered,
                self.radius_rate, radius_bias, self.gyro_bias,
            ] + list(command)
        )
        self.log_file.flush()

    def close_log(self):
        if self.log_file is not None:
            self.log_file.flush()
            self.log_file.close()
        self.log_file = None
        self.log_writer = None

    def shutdown(self):
        with self.lock:
            self.publish_command([0.0, 0.0, 0.0, 0.0])
            self.close_log()

    @staticmethod
    def elapsed(now, then):
        return (now - then).nanoseconds * 1e-9

    @staticmethod
    def clamp(value, lower, upper):
        return min(max(value, lower), upper)


def main(args=None):
    rclpy.init(args=args)
    node = NoSteerCircleDriftTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
