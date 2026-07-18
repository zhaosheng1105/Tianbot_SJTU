"""Hardware ROS 2 ports of the two MATLAB no-steer drift controllers.

The source MATLAB controller order is FL, FR, RL, RR.  Commands in this file
stay in that logical order until ``motor_torque_signs`` is applied immediately
before publishing a DmMitCommand.
"""

import csv
import math
import os
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from tianbot_core.msg import DmMitCommand, RoverMotionModeStatus
from tianbot_core.srv import SetControlMode


class MatlabDriftPid(Node):
    """Run either the MATLAB IMU-only or IMU+odom controller on Tianbot."""

    KIND_IMU_ONLY = "imu_only"
    KIND_IMU_ODOM = "imu_odom"

    PHASE_IDLE = 0
    PHASE_CALIBRATE = 1
    PHASE_ACCELERATE = 2
    PHASE_INITIATE = 3
    PHASE_HOLD = 4
    PHASE_STOP = 5
    PHASE_ABORT = 6

    PHASE_NAMES = {
        PHASE_IDLE: "idle",
        PHASE_CALIBRATE: "calibrate",
        PHASE_ACCELERATE: "accelerate",
        PHASE_INITIATE: "initiate",
        PHASE_HOLD: "hold",
        PHASE_STOP: "stop",
        PHASE_ABORT: "abort",
    }

    def __init__(self, controller_kind, node_name=None):
        if controller_kind not in (self.KIND_IMU_ONLY, self.KIND_IMU_ODOM):
            raise ValueError("unsupported controller kind: %s" % controller_kind)

        node_name = node_name or "drift_%s_pid" % controller_kind
        super().__init__(node_name)
        self.controller_kind = controller_kind
        self.uses_odom = controller_kind == self.KIND_IMU_ODOM
        self.lock = threading.RLock()

        self._read_common_parameters()
        if self.uses_odom:
            self._read_imu_odom_parameters()
        else:
            self._read_imu_only_parameters()
        self._validate_parameters()

        self.phase = self.PHASE_IDLE
        self.phase_start = self.get_clock().now()
        self.node_start_time = self.phase_start
        self.last_control_time = self.phase_start
        self.test_start = None
        self.auto_start_done = not self.auto_start
        self.auto_start_ready_since = None
        self.auto_start_wait_reason = ""
        self.mit_request_future = None
        self.mit_request_stamp = None
        self.mit_request_confirmed = not self.activate_mit_on_start

        self.last_imu_stamp = None
        self.last_odom_stamp = None
        self.mode_status_received = False
        self.mit_mode_ready = False

        self.imu_raw = 0.0
        self.imu_bias = 0.0
        self.imu_calibration_samples = []
        self.yaw_rate = 0.0
        self.previous_yaw_rate = 0.0
        self.yaw_rate_derivative = 0.0
        self.wrong_direction_duration = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_speed_raw = 0.0
        self.odom_speed = 0.0
        self.circle_center = None
        self.odom_radius_raw = float("nan")
        self.odom_radius = float("nan")
        self.previous_odom_radius = float("nan")
        self.radius_rate = 0.0

        self.yaw_hold = 0.0
        self.abort_reason = ""
        self.previous_command = [0.0, 0.0, 0.0, 0.0]
        self.last_yaw_ref = 0.0
        self.last_radius_bias = 0.0
        self.log_file = None
        self.log_writer = None
        self.log_path = ""

        self.cmd_pub = self.create_publisher(DmMitCommand, self.cmd_topic, 1)
        self.diag_pub = self.create_publisher(
            Float64MultiArray, self.diagnostics_topic, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.odom_sub = None
        if self.uses_odom:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                qos_profile_sensor_data,
            )
        self.mode_status_sub = self.create_subscription(
            RoverMotionModeStatus,
            self.mode_status_topic,
            self.mode_status_callback,
            10,
        )
        self.mode_client = self.create_client(
            SetControlMode, self.set_control_mode_service
        )
        self.start_service = self.create_service(
            Trigger, "~/start", self.start_callback
        )
        self.abort_service = self.create_service(
            Trigger, "~/abort", self.abort_callback
        )
        self.startup_timer = self.create_timer(0.10, self.startup_callback)
        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz, self.control_callback
        )

        feedback = self.feedback_description()
        self.get_logger().info(
            "%s ready: R=%.3f m, V=%.3f m/s, r=%.4f rad/s, turns=%.2f"
            % (
                feedback,
                self.target_radius,
                self.target_speed,
                self.nominal_yaw_rate,
                self.target_turns,
            )
        )
        self.get_logger().info(
            "IMU convention: right-handed, CCW positive; imu_yaw_sign=%+.0f"
            % self.imu_yaw_sign
        )
        self.get_logger().warning(
            "MATLAB wheel order is [FL, FR, RL, RR]; published motor signs are %s"
            % self.motor_torque_signs
        )
        self.get_logger().warning(
            "This launch uses the MATLAB torque profile (up to %.3f Nm/wheel). "
            "Keep the test area clear and keep the abort service ready."
            % self.software_torque_max
        )
        if self.auto_start:
            self.get_logger().warning(
                "AUTO START armed: after readiness checks there is a %.1f s "
                "safety delay followed by %.2f s stationary IMU calibration."
                % (self.auto_start_delay, self.calibration_time)
            )

    def feedback_description(self):
        if self.uses_odom:
            return "IMU yaw-rate PD + odom speed P + odom radius PD"
        return "IMU yaw-rate PD only"

    def _read_common_parameters(self):
        self.control_rate_hz = float(self.param("control_rate_hz", 100.0))
        self.imu_topic = self.param("imu_topic", "/tianbot/imu")
        self.odom_topic = "/tianbot/odom"
        if self.uses_odom:
            self.odom_topic = self.param("odom_topic", self.odom_topic)
        self.cmd_topic = self.param(
            "wheel_torque_command_topic", "/tianbot/wheel_mit_cmd"
        )
        self.diagnostics_topic = self.param(
            "diagnostics_topic", "/drift_pid/diagnostics"
        )
        self.mode_status_topic = self.param(
            "mode_status_topic", "/tianbot/motion_mode_status"
        )
        self.set_control_mode_service = self.param(
            "set_control_mode_service", "/tianbot/set_control_mode"
        )

        self.auto_start = bool(self.param("auto_start", True))
        self.activate_mit_on_start = bool(
            self.param("activate_mit_on_start", True)
        )
        self.require_mit_mode = bool(self.param("require_mit_mode", True))
        self.mit_activation_timeout = float(
            self.param("mit_activation_timeout_s", 15.0)
        )
        self.auto_start_delay = float(self.param("auto_start_delay_s", 3.0))

        self.motor_torque_signs = [
            float(value)
            for value in self.param(
                "motor_torque_signs", [1.0, 1.0, 1.0, 1.0]
            )
        ]
        self.imu_yaw_sign = (
            1.0 if float(self.param("imu_yaw_sign", 1.0)) >= 0.0 else -1.0
        )
        self.direction = (
            1.0 if float(self.param("direction", 1.0)) >= 0.0 else -1.0
        )
        self.target_radius = float(self.param("target_radius_m", 0.60))
        self.target_speed = float(self.param("target_speed_mps", 1.00))
        self.target_turns = float(self.param("target_turns", 3.0))

        self.calibration_time = float(self.param("calibration_time_s", 0.40))
        self.calibration_timeout = float(
            self.param("calibration_timeout_s", 2.0)
        )
        self.calibration_min_samples = int(
            self.param("calibration_min_samples", 20)
        )
        self.imu_filter_tau = float(self.param("imu_filter_tau_s", 0.020))
        self.imu_derivative_tau = float(
            self.param("imu_derivative_tau_s", 0.050)
        )

        self.pulse_duration = float(self.param("pulse_duration_s", 0.180))
        self.pulse_inner = float(
            self.param("pulse_inner_torque_nm", -0.258487313274)
        )
        self.pulse_outer = float(self.param("pulse_outer_torque_nm", 0.480))
        self.transition_time = float(self.param("transition_time_s", 0.18))
        self.hold_left_ff = float(
            self.param("hold_left_feedforward_nm", -0.214606873213331)
        )
        self.hold_right_ff = float(
            self.param("hold_right_feedforward_nm", 0.306833460264262)
        )
        self.max_yaw_correction = float(
            self.param("max_yaw_correction_nm", 0.18)
        )
        self.stop_time = float(self.param("stop_time_s", 0.45))
        self.software_torque_max = float(
            self.param("software_torque_max_nm", 0.48)
        )
        self.command_slew = float(self.param("command_slew_nmps", 3.0))

        self.sensor_timeout = float(self.param("sensor_timeout_s", 0.20))
        self.max_abs_yaw_rate = float(
            self.param("max_abs_yaw_rate_radps", 4.0)
        )
        self.max_odom_speed = float(self.param("max_odom_speed_mps", 2.5))
        self.wrong_direction_yaw_threshold = float(
            self.param("wrong_direction_yaw_threshold_radps", 0.20)
        )
        self.wrong_direction_timeout = float(
            self.param("wrong_direction_timeout_s", 0.30)
        )
        self.max_run_time = float(self.param("max_run_time_s", 0.0))
        self.log_directory = os.path.expanduser(
            self.param("log_directory", "~/.ros/drift_test_logs")
        )

    def _read_imu_only_parameters(self):
        self.accel_time = float(self.param("accel_time_s", 0.54))
        self.accel_rise_time = float(self.param("accel_rise_time_s", 0.12))
        self.accel_torque = float(self.param("accel_torque_nm", 0.100))
        self.yaw_kp = float(
            self.param("yaw_kp_nm_per_radps", 0.210315883222)
        )
        self.yaw_kd = float(
            self.param("yaw_kd_nm_per_radps2", 0.0249765583251)
        )

    def _read_imu_odom_parameters(self):
        self.accel_time = float(self.param("accel_time_s", 0.50))
        self.accel_feedforward = float(
            self.param("accel_feedforward_nm", 0.140)
        )
        self.accel_torque_max = float(self.param("accel_torque_max_nm", 0.29))
        self.pulse_speed_kp = float(
            self.param("pulse_speed_kp_nm_per_mps", 0.080)
        )
        self.speed_filter_tau = float(self.param("speed_filter_tau_s", 0.060))
        self.speed_kp = float(self.param("speed_kp_nm_per_mps", 0.300))
        self.max_speed_correction = float(
            self.param("max_speed_correction_nm", 0.18)
        )
        self.yaw_kp = float(self.param("yaw_kp_nm_per_radps", 0.240))
        self.yaw_kd = float(self.param("yaw_kd_nm_per_radps2", 0.010))
        self.center_lock_delay = float(
            self.param("center_lock_delay_s", 0.80)
        )
        self.radius_enable_delay = float(
            self.param("radius_enable_delay_s", 1.20)
        )
        self.radius_kp = float(
            self.param("radius_kp_radps_per_m", 0.20)
        )
        self.radius_kd = float(
            self.param("radius_kd_rad_per_m", 0.060)
        )
        self.radius_filter_tau = float(
            self.param("radius_filter_tau_s", 0.25)
        )
        self.radius_rate_tau = float(self.param("radius_rate_tau_s", 0.35))
        self.max_radius_yaw_bias = float(
            self.param("max_radius_yaw_bias_radps", 0.12)
        )

    def _validate_parameters(self):
        if len(self.motor_torque_signs) != 4:
            raise ValueError("motor_torque_signs must contain FL, FR, RL, RR")
        if any(abs(value) < 1.0e-9 for value in self.motor_torque_signs):
            raise ValueError("motor_torque_signs entries must be non-zero")
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        if self.target_radius <= 0.0 or self.target_speed <= 0.0:
            raise ValueError("target radius and speed must be positive")
        if self.target_turns <= 0.0:
            raise ValueError("target_turns must be positive")
        if self.software_torque_max <= 0.0 or self.command_slew <= 0.0:
            raise ValueError("torque limit and slew rate must be positive")
        if self.max_run_time <= 0.0:
            circle_time = 2.0 * math.pi * self.target_radius / self.target_speed
            self.max_run_time = (
                self.calibration_time
                + self.accel_time
                + self.pulse_duration
                + 1.40 * self.target_turns * circle_time
                + self.stop_time
            )

    def param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    @property
    def nominal_yaw_rate(self):
        return self.direction * self.target_speed / self.target_radius

    def imu_callback(self, msg):
        now = self.get_clock().now()
        raw = float(msg.angular_velocity.z)
        with self.lock:
            self.last_imu_stamp = now
            self.imu_raw = raw
            if self.phase == self.PHASE_CALIBRATE and math.isfinite(raw):
                self.imu_calibration_samples.append(raw)

    def odom_callback(self, msg):
        now = self.get_clock().now()
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self.lock:
            self.last_odom_stamp = now
            self.odom_x = float(msg.pose.pose.position.x)
            self.odom_y = float(msg.pose.pose.position.y)
            self.odom_yaw = yaw
            # The MATLAB controller explicitly uses odom body-x speed, not
            # hypot(vx, vy), because wheel odometry suppresses lateral slip.
            self.odom_speed_raw = max(0.0, float(msg.twist.twist.linear.x))

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

    def startup_callback(self):
        if not self.auto_start or self.auto_start_done:
            return
        now = self.get_clock().now()
        with self.lock:
            if self.phase != self.PHASE_IDLE:
                self.auto_start_done = True
                return

        if self.activate_mit_on_start and not self.mit_request_confirmed:
            self.handle_mit_activation(now)
            return

        with self.lock:
            ok, reason = self.sensors_ready(now)
            if not ok:
                self.auto_start_ready_since = None
                self.set_auto_start_wait_reason(reason)
                return

            self.set_auto_start_wait_reason("")
            if self.auto_start_ready_since is None:
                self.auto_start_ready_since = now
                self.get_logger().warning(
                    "MIT mode and required sensors are ready. Test sequence "
                    "starts in %.1f s; Ctrl-C or call ~/abort to cancel."
                    % self.auto_start_delay
                )
                return
            if self.elapsed(now, self.auto_start_ready_since) < self.auto_start_delay:
                return
            success, message = self.start_test_locked(now)
            if not success:
                self.fail_auto_start(message)

    def handle_mit_activation(self, now):
        if self.mit_request_future is None:
            if not self.mode_client.service_is_ready():
                self.set_auto_start_wait_reason(
                    "service %s" % self.set_control_mode_service
                )
                if (
                    self.elapsed(now, self.node_start_time)
                    > self.mit_activation_timeout
                ):
                    self.fail_auto_start("MIT mode service was unavailable")
                return
            request = SetControlMode.Request()
            request.mode = "mit"
            request.save_to_flash = False
            self.mit_request_stamp = now
            self.mit_request_future = self.mode_client.call_async(request)
            self.get_logger().info(
                "Requesting MIT mode from %s" % self.set_control_mode_service
            )
            return

        if not self.mit_request_future.done():
            if self.elapsed(now, self.mit_request_stamp) > self.mit_activation_timeout:
                self.fail_auto_start("MIT activation request timed out")
            return
        try:
            response = self.mit_request_future.result()
        except Exception as exc:
            self.fail_auto_start("MIT activation service failed: %s" % exc)
            return
        if response is None or not response.success:
            detail = "empty response" if response is None else response.message
            self.fail_auto_start("MIT activation rejected: %s" % detail)
            return
        self.mit_request_confirmed = True
        self.set_auto_start_wait_reason("")
        self.get_logger().info("MIT activation service confirmed")

    def set_auto_start_wait_reason(self, reason):
        if reason == self.auto_start_wait_reason:
            return
        self.auto_start_wait_reason = reason
        if reason:
            self.get_logger().info("Auto-start waiting for %s" % reason)

    def fail_auto_start(self, reason):
        self.auto_start_done = True
        self.auto_start_ready_since = None
        self.publish_command([0.0, 0.0, 0.0, 0.0])
        self.get_logger().error("AUTO START cancelled: %s" % reason)

    def start_callback(self, _request, response):
        now = self.get_clock().now()
        with self.lock:
            response.success, response.message = self.start_test_locked(now)
        return response

    def start_test_locked(self, now):
        if self.phase != self.PHASE_IDLE:
            return False, "test is already active"
        ok, reason = self.sensors_ready(now)
        if not ok:
            return False, reason

        self.test_start = now
        self.phase_start = now
        self.phase = self.PHASE_CALIBRATE
        self.auto_start_done = True
        self.abort_reason = ""
        self.imu_calibration_samples = []
        self.imu_bias = 0.0
        self.yaw_rate = 0.0
        self.previous_yaw_rate = 0.0
        self.yaw_rate_derivative = 0.0
        self.wrong_direction_duration = 0.0
        self.odom_speed = 0.0
        self.circle_center = None
        self.odom_radius_raw = float("nan")
        self.odom_radius = float("nan")
        self.previous_odom_radius = float("nan")
        self.radius_rate = 0.0
        self.yaw_hold = 0.0
        self.previous_command = [0.0, 0.0, 0.0, 0.0]
        self.last_control_time = now
        self.open_log(now)
        self.get_logger().warning(
            "Drift sequence STARTED in stationary IMU calibration phase"
        )
        return True, "drift sequence started"

    def abort_callback(self, _request, response):
        with self.lock:
            if self.phase == self.PHASE_IDLE:
                pending = self.auto_start and not self.auto_start_done
                self.auto_start_done = True
                self.auto_start_ready_since = None
                self.publish_command([0.0, 0.0, 0.0, 0.0])
                response.success = True
                response.message = (
                    "pending auto-start cancelled; zero command published"
                    if pending
                    else "already idle; zero command published"
                )
                return response
            self.enter_abort("manual abort")
            response.success = True
            response.message = "abort requested"
            return response

    def sensors_ready(self, now):
        if self.require_mit_mode and not self.mode_status_received:
            return False, "motion-mode status"
        if self.require_mit_mode and not self.mit_mode_ready:
            return False, "DYN PC_MIT/mit ready state"
        if self.last_imu_stamp is None:
            return False, "IMU data"
        if self.elapsed(now, self.last_imu_stamp) > self.sensor_timeout:
            return False, "fresh IMU data"
        if self.uses_odom:
            if self.last_odom_stamp is None:
                return False, "odometry data"
            if self.elapsed(now, self.last_odom_stamp) > self.sensor_timeout:
                return False, "fresh odometry data"
        return True, ""

    def control_callback(self):
        now = self.get_clock().now()
        dt = self.clamp(self.elapsed(now, self.last_control_time), 1.0e-4, 0.05)
        self.last_control_time = now

        with self.lock:
            if self.phase == self.PHASE_IDLE:
                self.publish_command([0.0, 0.0, 0.0, 0.0])
                self.publish_diagnostics()
                return

            ok, reason = self.sensors_ready(now)
            if not ok:
                self.enter_abort(reason)

            self.update_sensor_filters(dt)
            if self.phase != self.PHASE_ABORT:
                self.apply_safety_checks(now, dt)

            phase_time = self.elapsed(now, self.phase_start)
            raw_command = [0.0, 0.0, 0.0, 0.0]
            yaw_ref = 0.0
            radius_bias = 0.0

            if self.phase == self.PHASE_CALIBRATE:
                if (
                    phase_time >= self.calibration_time
                    and len(self.imu_calibration_samples)
                    >= self.calibration_min_samples
                ):
                    self.imu_bias = sum(self.imu_calibration_samples) / len(
                        self.imu_calibration_samples
                    )
                    self.yaw_rate = 0.0
                    self.previous_yaw_rate = 0.0
                    self.yaw_rate_derivative = 0.0
                    self.enter_phase(self.PHASE_ACCELERATE, now)
                    self.get_logger().info(
                        "IMU bias calibrated from %d messages: %.6f rad/s"
                        % (len(self.imu_calibration_samples), self.imu_bias)
                    )
                elif phase_time >= self.calibration_timeout:
                    self.enter_abort(
                        "IMU calibration received only %d/%d messages"
                        % (
                            len(self.imu_calibration_samples),
                            self.calibration_min_samples,
                        )
                    )

            elif self.phase == self.PHASE_ACCELERATE:
                yaw_ref = 0.0
                if self.uses_odom:
                    raw_command = self.imu_odom_accel_command(phase_time)
                else:
                    raw_command = self.imu_only_accel_command(phase_time)
                if phase_time >= self.accel_time:
                    self.enter_phase(self.PHASE_INITIATE, now)

            elif self.phase == self.PHASE_INITIATE:
                ramp = self.smoothstep(
                    self.clamp(phase_time / self.pulse_duration, 0.0, 1.0)
                )
                yaw_ref = self.nominal_yaw_rate * ramp
                trim = 0.0
                if self.uses_odom:
                    trim = self.clamp(
                        self.pulse_speed_kp * (self.target_speed - self.odom_speed),
                        -0.06,
                        0.06,
                    )
                raw_command = self.pulse_command(trim)
                if phase_time >= self.pulse_duration:
                    self.enter_phase(self.PHASE_HOLD, now)

            elif self.phase == self.PHASE_HOLD:
                if self.uses_odom:
                    if (
                        self.circle_center is None
                        and phase_time >= self.center_lock_delay
                    ):
                        self.set_circle_center_from_odom()
                    if self.circle_center is not None:
                        self.update_radius_estimate(dt)
                        self.yaw_hold += self.direction * self.yaw_rate * dt
                    radius_bias = self.calculate_radius_bias(phase_time)
                else:
                    self.yaw_hold += self.direction * self.yaw_rate * dt

                if self.yaw_hold >= 2.0 * math.pi * self.target_turns:
                    raw_command = self.hold_feedforward_command()
                    yaw_ref = 0.0
                    self.enter_phase(self.PHASE_STOP, now)
                else:
                    yaw_ref = self.nominal_yaw_rate + radius_bias
                    raw_command = self.hold_command(
                        phase_time, yaw_ref, radius_bias
                    )

            elif self.phase == self.PHASE_STOP:
                scale = max(0.0, 1.0 - phase_time / self.stop_time)
                raw_command = [
                    scale * value for value in self.hold_feedforward_command()
                ]
                if phase_time >= self.stop_time:
                    self.finish_test("completed")
                    return

            elif self.phase == self.PHASE_ABORT:
                raw_command = [0.0, 0.0, 0.0, 0.0]
                if phase_time >= 0.30:
                    self.finish_test("aborted: %s" % self.abort_reason)
                    return

            command = self.limit_and_slew(raw_command, dt)
            self.last_yaw_ref = yaw_ref
            self.last_radius_bias = radius_bias
            self.publish_command(command)
            self.publish_diagnostics()
            self.write_log(now, command)

    def update_sensor_filters(self, dt):
        if self.phase == self.PHASE_CALIBRATE:
            self.yaw_rate = 0.0
            self.previous_yaw_rate = 0.0
            self.yaw_rate_derivative = 0.0
        else:
            corrected = self.imu_yaw_sign * (self.imu_raw - self.imu_bias)
            alpha = dt / (self.imu_filter_tau + dt)
            self.yaw_rate += alpha * (corrected - self.yaw_rate)
            raw_derivative = (self.yaw_rate - self.previous_yaw_rate) / dt
            self.previous_yaw_rate = self.yaw_rate
            derivative_alpha = dt / (self.imu_derivative_tau + dt)
            self.yaw_rate_derivative += derivative_alpha * (
                raw_derivative - self.yaw_rate_derivative
            )

        if self.uses_odom:
            speed_alpha = dt / (self.speed_filter_tau + dt)
            self.odom_speed += speed_alpha * (
                self.odom_speed_raw - self.odom_speed
            )

    def apply_safety_checks(self, now, dt):
        if not math.isfinite(self.imu_raw) or not math.isfinite(self.yaw_rate):
            self.enter_abort("non-finite IMU value")
            return
        if abs(self.yaw_rate) > self.max_abs_yaw_rate:
            self.enter_abort("yaw-rate safety limit exceeded")
            return
        if self.uses_odom:
            odom_values = (
                self.odom_x,
                self.odom_y,
                self.odom_yaw,
                self.odom_speed_raw,
            )
            if not all(math.isfinite(value) for value in odom_values):
                self.enter_abort("non-finite odometry value")
                return
            if self.odom_speed > self.max_odom_speed:
                self.enter_abort("odometry speed safety limit exceeded")
                return
        if self.test_start is not None:
            if self.elapsed(now, self.test_start) > self.max_run_time:
                self.enter_abort("maximum run time exceeded")
                return

        if self.phase in (self.PHASE_INITIATE, self.PHASE_HOLD):
            signed_yaw_rate = self.direction * self.yaw_rate
            if signed_yaw_rate < -self.wrong_direction_yaw_threshold:
                self.wrong_direction_duration += dt
            else:
                self.wrong_direction_duration = 0.0
            if self.wrong_direction_duration >= self.wrong_direction_timeout:
                expected = "CCW/positive" if self.direction > 0.0 else "CW/negative"
                self.enter_abort(
                    "IMU yaw direction is opposite to requested %s motion" % expected
                )
        else:
            self.wrong_direction_duration = 0.0

    def imu_only_accel_command(self, phase_time):
        ramp = self.smoothstep(
            self.clamp(phase_time / self.accel_rise_time, 0.0, 1.0)
        )
        base = self.accel_torque * ramp
        correction = self.clamp(
            self.yaw_kp * (-self.yaw_rate)
            - self.yaw_kd * self.yaw_rate_derivative,
            -0.06,
            0.06,
        )
        return self.left_right_command(
            base - self.direction * correction,
            base + self.direction * correction,
        )

    def imu_odom_accel_command(self, phase_time):
        ramp = self.smoothstep(
            self.clamp(phase_time / self.accel_time, 0.0, 1.0)
        )
        speed_reference = self.target_speed * ramp
        base = self.clamp(
            self.accel_feedforward
            + self.speed_kp * (speed_reference - self.odom_speed),
            0.0,
            self.accel_torque_max,
        )
        correction = self.clamp(
            self.yaw_kp * (-self.yaw_rate)
            - self.yaw_kd * self.yaw_rate_derivative,
            -0.06,
            0.06,
        )
        return self.left_right_command(
            base - self.direction * correction,
            base + self.direction * correction,
        )

    def pulse_command(self, trim):
        base = 0.5 * (self.pulse_inner + self.pulse_outer) + trim
        difference = 0.5 * (self.pulse_outer - self.pulse_inner)
        return self.left_right_command(
            base - self.direction * difference,
            base + self.direction * difference,
        )

    def hold_feedforward_command(self):
        base = 0.5 * (self.hold_left_ff + self.hold_right_ff)
        difference = 0.5 * (self.hold_right_ff - self.hold_left_ff)
        return self.left_right_command(
            base - self.direction * difference,
            base + self.direction * difference,
        )

    def hold_command(self, hold_time, yaw_ref, _radius_bias):
        yaw_error = yaw_ref - self.yaw_rate
        yaw_correction = self.clamp(
            self.yaw_kp * yaw_error
            - self.yaw_kd * self.yaw_rate_derivative,
            -self.max_yaw_correction,
            self.max_yaw_correction,
        )
        speed_correction = 0.0
        if self.uses_odom:
            speed_correction = self.clamp(
                self.speed_kp * (self.target_speed - self.odom_speed),
                -self.max_speed_correction,
                self.max_speed_correction,
            )

        base = 0.5 * (self.hold_left_ff + self.hold_right_ff)
        base += speed_correction
        difference = 0.5 * (self.hold_right_ff - self.hold_left_ff)
        difference += yaw_correction
        hold = self.left_right_command(
            base - self.direction * difference,
            base + self.direction * difference,
        )
        blend = self.smoothstep(
            self.clamp(hold_time / self.transition_time, 0.0, 1.0)
        )
        pulse = self.pulse_command(0.0)
        return [
            (1.0 - blend) * pulse_value + blend * hold_value
            for pulse_value, hold_value in zip(pulse, hold)
        ]

    @staticmethod
    def left_right_command(left, right):
        return [left, right, left, right]

    def set_circle_center_from_odom(self):
        normal_x = -math.sin(self.odom_yaw)
        normal_y = math.cos(self.odom_yaw)
        self.circle_center = (
            self.odom_x + self.direction * self.target_radius * normal_x,
            self.odom_y + self.direction * self.target_radius * normal_y,
        )
        self.get_logger().info(
            "Odom circle center locked at (%.4f, %.4f)"
            % (self.circle_center[0], self.circle_center[1])
        )

    def update_radius_estimate(self, dt):
        dx = self.odom_x - self.circle_center[0]
        dy = self.odom_y - self.circle_center[1]
        self.odom_radius_raw = math.hypot(dx, dy)
        if not math.isfinite(self.odom_radius):
            self.odom_radius = self.odom_radius_raw
            self.previous_odom_radius = self.odom_radius_raw
            self.radius_rate = 0.0
            return
        alpha = dt / (self.radius_filter_tau + dt)
        self.odom_radius += alpha * (self.odom_radius_raw - self.odom_radius)
        raw_rate = (self.odom_radius - self.previous_odom_radius) / dt
        self.previous_odom_radius = self.odom_radius
        rate_alpha = dt / (self.radius_rate_tau + dt)
        self.radius_rate += rate_alpha * (raw_rate - self.radius_rate)

    def calculate_radius_bias(self, hold_time):
        if hold_time < self.radius_enable_delay:
            return 0.0
        if not math.isfinite(self.odom_radius):
            return 0.0
        magnitude = self.radius_kp * (self.odom_radius - self.target_radius)
        magnitude += self.radius_kd * self.radius_rate
        magnitude = self.clamp(
            magnitude, -self.max_radius_yaw_bias, self.max_radius_yaw_bias
        )
        return self.direction * magnitude

    def limit_and_slew(self, raw_command, dt):
        limited = [
            self.clamp(
                float(value), -self.software_torque_max, self.software_torque_max
            )
            for value in raw_command
        ]
        max_step = self.command_slew * dt
        output = [
            previous
            + self.clamp(target - previous, -max_step, max_step)
            for target, previous in zip(limited, self.previous_command)
        ]
        self.previous_command = output
        return output

    def publish_command(self, logical_command):
        msg = DmMitCommand()
        msg.p_des = [0.0, 0.0, 0.0, 0.0]
        msg.v_des = [0.0, 0.0, 0.0, 0.0]
        msg.kp = [0.0, 0.0, 0.0, 0.0]
        msg.kd = [0.0, 0.0, 0.0, 0.0]
        msg.t_ff = [
            float(torque * sign)
            for torque, sign in zip(logical_command, self.motor_torque_signs)
        ]
        self.cmd_pub.publish(msg)

    def enter_phase(self, phase, now):
        self.phase = phase
        self.phase_start = now
        self.get_logger().info("Drift phase: %s" % self.PHASE_NAMES[phase])

    def enter_abort(self, reason):
        if self.phase == self.PHASE_ABORT:
            return
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
        self.get_logger().warning(
            "Drift test finished: %s; integrated yaw=%.3f turns"
            % (result, self.yaw_hold / (2.0 * math.pi))
        )

    def publish_diagnostics(self):
        command = self.previous_command
        msg = Float64MultiArray()
        labels = (
            "phase,imu_raw,imu_bias,yaw_rate,yaw_rate_derivative,yaw_ref,"
            "yaw_turns,odom_speed,odom_radius_raw,odom_radius,radius_rate,"
            "radius_yaw_bias,odom_x,odom_y,torque_FL,torque_FR,torque_RL,"
            "torque_RR"
        )
        msg.layout.dim = [
            MultiArrayDimension(label=labels, size=18, stride=18)
        ]
        msg.data = [
            float(self.phase),
            self.imu_raw,
            self.imu_bias,
            self.yaw_rate,
            self.yaw_rate_derivative,
            self.last_yaw_ref,
            self.yaw_hold / (2.0 * math.pi),
            self.odom_speed if self.uses_odom else float("nan"),
            self.odom_radius_raw,
            self.odom_radius,
            self.radius_rate,
            self.last_radius_bias,
            self.odom_x if self.uses_odom else float("nan"),
            self.odom_y if self.uses_odom else float("nan"),
        ] + list(command)
        self.diag_pub.publish(msg)

    def open_log(self, now):
        os.makedirs(self.log_directory, exist_ok=True)
        stamp = now.nanoseconds * 1.0e-9
        filename = "drift_%s_pid_%.3f.csv" % (self.controller_kind, stamp)
        self.log_path = os.path.join(self.log_directory, filename)
        self.log_file = open(self.log_path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(
            [
                "time_s",
                "phase",
                "imu_raw_radps",
                "imu_bias_radps",
                "yaw_rate_radps",
                "yaw_rate_derivative_radps2",
                "yaw_ref_radps",
                "yaw_turns",
                "odom_speed_mps",
                "odom_x_m",
                "odom_y_m",
                "odom_yaw_rad",
                "odom_radius_raw_m",
                "odom_radius_m",
                "radius_rate_mps",
                "radius_yaw_bias_radps",
                "logical_FL_Nm",
                "logical_FR_Nm",
                "logical_RL_Nm",
                "logical_RR_Nm",
                "published_FL_Nm",
                "published_FR_Nm",
                "published_RL_Nm",
                "published_RR_Nm",
            ]
        )
        self.get_logger().info("Logging to %s" % self.log_path)

    def write_log(self, now, command):
        if self.log_writer is None or self.test_start is None:
            return
        published = [
            torque * sign
            for torque, sign in zip(command, self.motor_torque_signs)
        ]
        self.log_writer.writerow(
            [
                self.elapsed(now, self.test_start),
                self.PHASE_NAMES[self.phase],
                self.imu_raw,
                self.imu_bias,
                self.yaw_rate,
                self.yaw_rate_derivative,
                self.last_yaw_ref,
                self.yaw_hold / (2.0 * math.pi),
                self.odom_speed if self.uses_odom else float("nan"),
                self.odom_x if self.uses_odom else float("nan"),
                self.odom_y if self.uses_odom else float("nan"),
                self.odom_yaw if self.uses_odom else float("nan"),
                self.odom_radius_raw,
                self.odom_radius,
                self.radius_rate,
                self.last_radius_bias,
            ]
            + list(command)
            + published
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
        return (now - then).nanoseconds * 1.0e-9

    @staticmethod
    def clamp(value, lower, upper):
        return min(max(value, lower), upper)

    @staticmethod
    def smoothstep(value):
        value = min(max(value, 0.0), 1.0)
        return value * value * (3.0 - 2.0 * value)


def run_controller(controller_kind, args=None):
    rclpy.init(args=args)
    node = MatlabDriftPid(controller_kind)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main_imu_only(args=None):
    run_controller(MatlabDriftPid.KIND_IMU_ONLY, args=args)


def main_imu_odom(args=None):
    run_controller(MatlabDriftPid.KIND_IMU_ODOM, args=args)
