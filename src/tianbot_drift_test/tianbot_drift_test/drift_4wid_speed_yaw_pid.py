"""Hardware port of ``drift_4wid_independent_speed_yaw_pid.m``.

The controller keeps the MATLAB logical wheel order FL, FR, RL, RR.  It ports
the MATLAB sensor filters and the accelerometer-derived sideslip outer loop;
the nonlinear tire equilibrium itself remains an offline calculation whose
four-wheel result is supplied through ROS parameters.
"""

import csv
import math
import os

import rclpy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from tianbot_drift_test.matlab_drift_pid import MatlabDriftPid


class Drift4WidSpeedYawPid(MatlabDriftPid):
    """Four-wheel-independent speed/yaw PID for the Tianbot DYN chassis."""

    def __init__(self):
        super().__init__(node_name="drift_4wid_speed_yaw_pid")

        self.previous_yaw_filtered = 0.0
        self.previous_speed_filtered = 0.0
        self.yaw_filter_stamp = None
        self.speed_filter_stamp = None
        self.speed_derivative = 0.0
        self.accel_x_raw = 0.0
        self.accel_y_raw = 0.0
        self.accel_x_bias = 0.0
        self.accel_y_bias = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_x_calibration_samples = []
        self.accel_y_calibration_samples = []
        self.beta_estimate = 0.0
        self.speed_integral = 0.0
        self.yaw_integral = 0.0
        self.last_speed_ref = 0.0
        self.last_speed_error = 0.0
        self.last_yaw_error = 0.0
        self.last_speed_correction = 0.0
        self.last_yaw_correction = 0.0
        self.last_raw_command = [0.0, 0.0, 0.0, 0.0]

        self.get_logger().info(
            "MATLAB filters enabled: yaw tau=%.3f/%.3f s, speed "
            "tau=%.3f/%.3f s (signal/derivative)."
            % (
                self.imu_filter_tau,
                self.imu_derivative_tau,
                self.speed_filter_tau,
                self.speed_derivative_tau,
            )
        )
        self.get_logger().info(
            "MATLAB nominal independent hold torque [FL, FR, RL, RR] = %s Nm"
            % self.hold_feedforward
        )

    def feedback_description(self):
        return (
            "filtered IMU yaw-rate PID + filtered odom speed PID + "
            "accelerometer sideslip outer loop + independent 4WID allocation"
        )

    def _read_common_parameters(self):
        self.control_rate_hz = float(self.param("control_rate_hz", 100.0))
        self.imu_topic = self.param("imu_topic", "/tianbot/imu")
        self.odom_topic = self.param("odom_topic", "/tianbot/odom")
        self.cmd_topic = self.param(
            "wheel_torque_command_topic", "/tianbot/wheel_mit_cmd"
        )
        self.diagnostics_topic = self.param(
            "diagnostics_topic", "/drift_4wid_speed_yaw_pid/diagnostics"
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
            1.0
            if float(self.param("imu_yaw_sign", 1.0)) >= 0.0
            else -1.0
        )
        self.direction = (
            1.0 if float(self.param("direction", 1.0)) >= 0.0 else -1.0
        )

        self.target_radius = float(self.param("target_radius_m", 1.0))
        self.target_speed = float(self.param("target_speed_mps", 2.0))
        self.target_turns = float(self.param("target_turns", 5.0))

        self.calibration_time = float(self.param("calibration_time_s", 2.00))
        self.calibration_timeout = float(
            self.param("calibration_timeout_s", 3.0)
        )
        self.calibration_min_samples = int(
            self.param("calibration_min_samples", 20)
        )

        self.stop_time = float(self.param("stop_time_s", 0.55))
        self.software_torque_max = float(
            self.param("software_torque_max_nm", 0.68)
        )
        self.command_slew = float(self.param("command_slew_nmps", 4.0))

        self.sensor_timeout = float(self.param("sensor_timeout_s", 0.20))
        self.max_abs_yaw_rate = float(
            self.param("max_abs_yaw_rate_radps", 5.0)
        )
        self.max_odom_speed = float(self.param("max_odom_speed_mps", 3.5))
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

    def _read_controller_parameters(self):
        self.vehicle_mass = float(self.param("vehicle_mass_kg", 7.0))
        self.wheel_radius = float(self.param("wheel_radius_m", 0.048))

        self.imu_filter_tau = float(self.param("imu_filter_tau_s", 0.020))
        self.imu_derivative_tau = float(
            self.param("imu_derivative_tau_s", 0.050)
        )
        self.speed_filter_tau = float(
            self.param("speed_filter_tau_s", 0.060)
        )
        self.speed_derivative_tau = float(
            self.param("speed_derivative_tau_s", 0.100)
        )
        self.accel_x_sign = (
            1.0
            if float(self.param("accel_x_sign", 1.0)) >= 0.0
            else -1.0
        )
        self.accel_y_sign = (
            1.0
            if float(self.param("accel_y_sign", 1.0)) >= 0.0
            else -1.0
        )
        self.beta_desired = math.radians(
            float(self.param("beta_desired_deg", -30.0))
        )
        self.beta_kp = float(
            self.param("beta_kp_radps_per_rad", 1.5)
        )
        self.max_beta_yaw_ref_correction = float(
            self.param("max_beta_yaw_ref_correction_radps", 0.40)
        )
        self.beta_min = math.radians(
            float(self.param("beta_estimate_min_deg", -70.0))
        )
        self.beta_max = math.radians(
            float(self.param("beta_estimate_max_deg", 30.0))
        )
        self.beta_min_speed = float(
            self.param("beta_estimate_min_speed_mps", 0.35)
        )

        self.accel_time = float(self.param("accel_time_s", 0.40))
        self.accel_torque_max = float(
            self.param("accel_torque_max_nm", 0.68)
        )
        self.accel_max_yaw_correction = float(
            self.param("accel_max_yaw_correction_nm", 0.08)
        )
        self.pulse_duration = float(self.param("pulse_duration_s", 0.82))
        self.pulse_torque = [
            float(value)
            for value in self.param(
                "pulse_torque_nm", [-0.128, 0.260, -0.096, 0.248]
            )
        ]
        self.transition_time = float(self.param("transition_time_s", 0.05))

        self.hold_feedforward = [
            float(value)
            for value in self.param(
                "hold_feedforward_nm",
                [
                    0.0796263991840194,
                    -0.555616494418756,
                    0.614939850009095,
                    0.575811875532933,
                ],
            )
        ]
        self.speed_weights = [
            float(value)
            for value in self.param(
                "speed_distribution_weights",
                [
                    1.0,
                    1.0,
                    1.0,
                    1.0,
                ],
            )
        ]
        self.yaw_vector = [
            float(value)
            for value in self.param(
                "yaw_distribution_vector",
                [
                    -1.0,
                    1.0,
                    -1.0,
                    1.0,
                ],
            )
        ]

        self.speed_kp = float(self.param("speed_kp_nm_per_mps", 0.24))
        self.speed_ki = float(self.param("speed_ki_nm_per_m", 0.10))
        self.speed_kd = float(self.param("speed_kd_nm_per_mps2", 0.012))
        self.max_speed_correction = float(
            self.param("max_speed_correction_nm", 0.22)
        )
        self.speed_integral_limit = float(
            self.param("speed_integral_limit_m", 1.2)
        )

        self.yaw_kp = float(self.param("yaw_kp_nm_per_radps", 0.30))
        self.yaw_ki = float(self.param("yaw_ki_nm_per_rad", 0.080))
        self.yaw_kd = float(self.param("yaw_kd_nm_per_radps2", 0.025))
        self.max_yaw_correction = float(
            self.param("max_yaw_correction_nm", 0.30)
        )
        self.yaw_integral_limit = float(
            self.param("yaw_integral_limit_rad", 2.0)
        )

    def _validate_parameters(self):
        four_wheel_values = {
            "motor_torque_signs": self.motor_torque_signs,
            "pulse_torque_nm": self.pulse_torque,
            "hold_feedforward_nm": self.hold_feedforward,
            "speed_distribution_weights": self.speed_weights,
            "yaw_distribution_vector": self.yaw_vector,
        }
        for name, values in four_wheel_values.items():
            if len(values) != 4:
                raise ValueError("%s must contain FL, FR, RL, RR" % name)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("%s entries must be finite" % name)
        if any(abs(value) < 1.0e-9 for value in self.motor_torque_signs):
            raise ValueError("motor_torque_signs entries must be non-zero")
        if self.direction != 1.0:
            raise ValueError(
                "this MATLAB equilibrium and pulse were validated only for "
                "CCW direction=+1"
            )
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        if self.target_radius <= 0.0 or self.target_speed <= 0.0:
            raise ValueError("target radius and speed must be positive")
        if self.target_turns <= 0.0:
            raise ValueError("target_turns must be positive")
        if self.calibration_time <= 0.0 or self.calibration_min_samples <= 0:
            raise ValueError("stationary IMU calibration must be enabled")
        if self.accel_time <= 0.0 or self.pulse_duration <= 0.0:
            raise ValueError("acceleration and pulse durations must be positive")
        if self.transition_time <= 0.0 or self.stop_time <= 0.0:
            raise ValueError("transition and stop durations must be positive")
        if self.software_torque_max <= 0.0 or self.command_slew <= 0.0:
            raise ValueError("torque limit and slew rate must be positive")
        if self.vehicle_mass <= 0.0 or self.wheel_radius <= 0.0:
            raise ValueError("vehicle mass and wheel radius must be positive")
        filter_taus = (
            self.imu_filter_tau,
            self.imu_derivative_tau,
            self.speed_filter_tau,
            self.speed_derivative_tau,
        )
        if any(value < 0.0 for value in filter_taus):
            raise ValueError("sensor filter time constants must be non-negative")
        if self.beta_min >= self.beta_max or self.beta_min_speed < 0.0:
            raise ValueError("invalid sideslip estimator bounds")
        if self.beta_kp < 0.0 or self.max_beta_yaw_ref_correction < 0.0:
            raise ValueError("sideslip outer-loop gains must be non-negative")
        if self.speed_integral_limit <= 0.0 or self.yaw_integral_limit <= 0.0:
            raise ValueError("integral limits must be positive")
        if self.max_run_time <= 0.0:
            circle_time = (
                2.0 * math.pi * self.target_radius / self.target_speed
            )
            self.max_run_time = (
                self.calibration_time
                + self.accel_time
                + self.pulse_duration
                + 1.45 * self.target_turns * circle_time
                + self.stop_time
            )

    def start_test_locked(self, now):
        success, message = super().start_test_locked(now)
        if success:
            self.previous_yaw_filtered = 0.0
            self.previous_speed_filtered = 0.0
            self.yaw_filter_stamp = None
            self.speed_filter_stamp = None
            self.yaw_rate = 0.0
            self.yaw_rate_derivative = 0.0
            self.odom_speed = 0.0
            self.speed_derivative = 0.0
            self.accel_x_bias = 0.0
            self.accel_y_bias = 0.0
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_x_calibration_samples = []
            self.accel_y_calibration_samples = []
            self.beta_estimate = 0.0
            self.speed_integral = 0.0
            self.yaw_integral = 0.0
            self.last_speed_ref = 0.0
            self.last_speed_error = 0.0
            self.last_yaw_error = 0.0
            self.last_speed_correction = 0.0
            self.last_yaw_correction = 0.0
            self.last_raw_command = [0.0, 0.0, 0.0, 0.0]
        return success, message

    @staticmethod
    def low_pass(previous, sample, dt, time_constant):
        if time_constant <= 0.0:
            return sample
        alpha = dt / (time_constant + dt)
        return previous + alpha * (sample - previous)

    def imu_callback(self, msg):
        now = self.get_clock().now()
        raw = float(msg.angular_velocity.z)
        accel_x_raw = float(msg.linear_acceleration.x)
        accel_y_raw = float(msg.linear_acceleration.y)
        with self.lock:
            self.last_imu_stamp = now
            self.imu_raw = raw
            self.accel_x_raw = accel_x_raw
            self.accel_y_raw = accel_y_raw
            if self.phase == self.PHASE_CALIBRATE:
                if math.isfinite(raw):
                    self.imu_calibration_samples.append(raw)
                if math.isfinite(accel_x_raw):
                    self.accel_x_calibration_samples.append(accel_x_raw)
                if math.isfinite(accel_y_raw):
                    self.accel_y_calibration_samples.append(accel_y_raw)
                self.yaw_rate = 0.0
                self.yaw_rate_derivative = 0.0
                self.previous_yaw_filtered = 0.0
                self.yaw_filter_stamp = now
                self.accel_x = 0.0
                self.accel_y = 0.0
                return
            if not all(
                math.isfinite(value)
                for value in (raw, accel_x_raw, accel_y_raw)
            ):
                self.yaw_rate = raw
                self.yaw_rate_derivative = raw
                self.accel_x = accel_x_raw
                self.accel_y = accel_y_raw
                return

            corrected = self.imu_yaw_sign * (raw - self.imu_bias)
            sample_dt = 1.0 / self.control_rate_hz
            if self.yaw_filter_stamp is not None:
                sample_dt = self.clamp(
                    self.elapsed(now, self.yaw_filter_stamp), 1.0e-4, 0.10
                )
            filtered = self.low_pass(
                self.yaw_rate, corrected, sample_dt, self.imu_filter_tau
            )
            raw_derivative = (
                (filtered - self.previous_yaw_filtered) / sample_dt
            )
            self.yaw_rate_derivative = self.low_pass(
                self.yaw_rate_derivative,
                raw_derivative,
                sample_dt,
                self.imu_derivative_tau,
            )
            self.yaw_rate = filtered
            self.previous_yaw_filtered = filtered
            self.yaw_filter_stamp = now
            self.accel_x = self.accel_x_sign * (
                accel_x_raw - self.accel_x_bias
            )
            self.accel_y = self.accel_y_sign * (
                accel_y_raw - self.accel_y_bias
            )

    def odom_callback(self, msg):
        now = self.get_clock().now()
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        speed = math.hypot(
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        with self.lock:
            self.last_odom_stamp = now
            self.odom_x = float(msg.pose.pose.position.x)
            self.odom_y = float(msg.pose.pose.position.y)
            self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.odom_speed_raw = speed

            sample_dt = 1.0 / self.control_rate_hz
            if self.speed_filter_stamp is not None:
                sample_dt = self.clamp(
                    self.elapsed(now, self.speed_filter_stamp), 1.0e-4, 0.10
                )
            filtered = self.low_pass(
                self.odom_speed, speed, sample_dt, self.speed_filter_tau
            )
            raw_derivative = (
                (filtered - self.previous_speed_filtered) / sample_dt
            )
            self.speed_derivative = self.low_pass(
                self.speed_derivative,
                raw_derivative,
                sample_dt,
                self.speed_derivative_tau,
            )
            self.odom_speed = filtered
            self.previous_speed_filtered = filtered
            self.speed_filter_stamp = now

    def control_callback(self):
        now = self.get_clock().now()
        dt = self.clamp(
            self.elapsed(now, self.last_control_time), 1.0e-4, 0.05
        )
        self.last_control_time = now

        with self.lock:
            if self.phase == self.PHASE_IDLE:
                self.publish_command([0.0, 0.0, 0.0, 0.0])
                self.publish_diagnostics()
                return

            ok, reason = self.sensors_ready(now)
            if not ok:
                self.enter_abort(reason)
            if self.phase != self.PHASE_ABORT:
                self.apply_safety_checks(now, dt)

            if self.phase == self.PHASE_CALIBRATE:
                self.beta_estimate = 0.0
            elif (
                self.odom_speed > self.beta_min_speed
                and all(
                    math.isfinite(value)
                    for value in (
                        self.accel_x,
                        self.accel_y,
                        self.yaw_rate,
                    )
                )
            ):
                beta_dot = (
                    self.accel_y * math.cos(self.beta_estimate)
                    - self.accel_x * math.sin(self.beta_estimate)
                ) / self.odom_speed - self.yaw_rate
                self.beta_estimate = self.clamp(
                    self.beta_estimate + dt * beta_dot,
                    self.beta_min,
                    self.beta_max,
                )

            phase_time = self.elapsed(now, self.phase_start)
            raw_command = [0.0, 0.0, 0.0, 0.0]
            speed_ref = 0.0
            yaw_ref = 0.0
            speed_error = -self.odom_speed
            yaw_error = -self.yaw_rate
            speed_correction = 0.0
            yaw_correction = 0.0
            integrate = False

            if self.phase == self.PHASE_CALIBRATE:
                if (
                    phase_time >= self.calibration_time
                    and len(self.imu_calibration_samples)
                    >= self.calibration_min_samples
                    and len(self.accel_x_calibration_samples)
                    >= self.calibration_min_samples
                    and len(self.accel_y_calibration_samples)
                    >= self.calibration_min_samples
                ):
                    self.imu_bias = sum(self.imu_calibration_samples) / len(
                        self.imu_calibration_samples
                    )
                    self.accel_x_bias = sum(
                        self.accel_x_calibration_samples
                    ) / len(self.accel_x_calibration_samples)
                    self.accel_y_bias = sum(
                        self.accel_y_calibration_samples
                    ) / len(self.accel_y_calibration_samples)
                    self.yaw_rate = 0.0
                    self.yaw_rate_derivative = 0.0
                    self.previous_yaw_filtered = 0.0
                    self.yaw_filter_stamp = now
                    self.speed_derivative = 0.0
                    self.previous_speed_filtered = self.odom_speed
                    self.speed_filter_stamp = self.last_odom_stamp
                    self.beta_estimate = 0.0
                    self.enter_phase(self.PHASE_ACCELERATE, now)
                    self.get_logger().info(
                        "Stationary IMU calibration (%d samples): gyro="
                        "%.6f rad/s, accel_x=%.6f, accel_y=%.6f m/s^2"
                        % (
                            len(self.imu_calibration_samples),
                            self.imu_bias,
                            self.accel_x_bias,
                            self.accel_y_bias,
                        )
                    )
                elif phase_time >= self.calibration_timeout:
                    self.enter_abort(
                        "IMU calibration received gyro/x/y samples "
                        "%d/%d/%d (need %d each)"
                        % (
                            len(self.imu_calibration_samples),
                            len(self.accel_x_calibration_samples),
                            len(self.accel_y_calibration_samples),
                            self.calibration_min_samples,
                        )
                    )

            elif self.phase == self.PHASE_ACCELERATE:
                s = self.clamp(phase_time / self.accel_time, 0.0, 1.0)
                speed_ref = self.target_speed * self.smoothstep(s)
                speed_ref_derivative = (
                    self.target_speed
                    / self.accel_time
                    * self.dsmoothstep(s)
                )
                speed_error = speed_ref - self.odom_speed
                yaw_error = -self.yaw_rate
                acceleration_ff = (
                    self.vehicle_mass
                    * speed_ref_derivative
                    * self.wheel_radius
                    / 4.0
                )
                speed_correction = (
                    self.speed_kp * speed_error
                    - self.speed_kd * self.speed_derivative
                )
                yaw_correction = self.clamp(
                    self.yaw_kp * yaw_error
                    - self.yaw_kd * self.yaw_rate_derivative,
                    -self.accel_max_yaw_correction,
                    self.accel_max_yaw_correction,
                )
                base = self.clamp(
                    acceleration_ff + speed_correction,
                    0.0,
                    self.accel_torque_max,
                )
                raw_command = [
                    base + yaw_correction * weight
                    for weight in self.yaw_vector
                ]
                if phase_time >= self.accel_time:
                    self.enter_phase(self.PHASE_INITIATE, now)

            elif self.phase == self.PHASE_INITIATE:
                speed_ref = self.target_speed
                yaw_ref = self.nominal_yaw_rate * self.smoothstep(
                    self.clamp(
                        phase_time / self.pulse_duration, 0.0, 1.0
                    )
                )
                speed_error = speed_ref - self.odom_speed
                yaw_error = yaw_ref - self.yaw_rate
                raw_command = list(self.pulse_torque)
                self.speed_integral = 0.0
                self.yaw_integral = 0.0
                if phase_time >= self.pulse_duration:
                    self.enter_phase(self.PHASE_HOLD, now)

            elif self.phase == self.PHASE_HOLD:
                self.yaw_hold += self.direction * self.yaw_rate * dt
                if self.yaw_hold >= 2.0 * math.pi * self.target_turns:
                    raw_command = list(self.hold_feedforward)
                    self.enter_phase(self.PHASE_STOP, now)
                else:
                    speed_ref = self.target_speed
                    beta_yaw_correction = self.clamp(
                        self.beta_kp
                        * (self.beta_estimate - self.beta_desired),
                        -self.max_beta_yaw_ref_correction,
                        self.max_beta_yaw_ref_correction,
                    )
                    yaw_ref = self.nominal_yaw_rate + beta_yaw_correction
                    speed_error = speed_ref - self.odom_speed
                    yaw_error = yaw_ref - self.yaw_rate
                    speed_correction = self.clamp(
                        self.speed_kp * speed_error
                        + self.speed_ki * self.speed_integral
                        - self.speed_kd * self.speed_derivative,
                        -self.max_speed_correction,
                        self.max_speed_correction,
                    )
                    yaw_correction = self.clamp(
                        self.yaw_kp * yaw_error
                        + self.yaw_ki * self.yaw_integral
                        - self.yaw_kd * self.yaw_rate_derivative,
                        -self.max_yaw_correction,
                        self.max_yaw_correction,
                    )
                    hold_command = [
                        feedforward
                        + speed_correction * speed_weight
                        + yaw_correction * yaw_weight
                        for feedforward, speed_weight, yaw_weight in zip(
                            self.hold_feedforward,
                            self.speed_weights,
                            self.yaw_vector,
                        )
                    ]
                    blend = self.smoothstep(
                        self.clamp(
                            phase_time / self.transition_time, 0.0, 1.0
                        )
                    )
                    raw_command = [
                        (1.0 - blend) * pulse + blend * hold
                        for pulse, hold in zip(
                            self.pulse_torque, hold_command
                        )
                    ]
                    integrate = True

            elif self.phase == self.PHASE_STOP:
                scale = max(0.0, 1.0 - phase_time / self.stop_time)
                raw_command = [
                    scale * value for value in self.hold_feedforward
                ]
                if phase_time >= self.stop_time:
                    self.finish_test("completed")
                    return

            elif self.phase == self.PHASE_ABORT:
                raw_command = [0.0, 0.0, 0.0, 0.0]
                if phase_time >= 0.30:
                    self.finish_test("aborted: %s" % self.abort_reason)
                    return

            if integrate:
                self.speed_integral = self.clamp(
                    self.speed_integral + speed_error * dt,
                    -self.speed_integral_limit,
                    self.speed_integral_limit,
                )
                self.yaw_integral = self.clamp(
                    self.yaw_integral + yaw_error * dt,
                    -self.yaw_integral_limit,
                    self.yaw_integral_limit,
                )

            command = self.limit_and_slew(raw_command, dt)
            self.last_raw_command = list(raw_command)
            self.last_speed_ref = speed_ref
            self.last_yaw_ref = yaw_ref
            self.last_speed_error = speed_error
            self.last_yaw_error = yaw_error
            self.last_speed_correction = speed_correction
            self.last_yaw_correction = yaw_correction
            self.publish_command(command)
            self.publish_diagnostics()
            self.write_log(now, command)

    def publish_diagnostics(self):
        command = self.previous_command
        labels = (
            "phase,imu_raw,imu_bias,yaw_rate_filtered,"
            "yaw_rate_derivative_filtered,yaw_ref,yaw_error,yaw_turns,"
            "odom_speed_raw,odom_speed_filtered,"
            "odom_speed_derivative_filtered,speed_ref,speed_error,"
            "speed_integral,yaw_integral,speed_correction,yaw_correction,"
            "accel_x_raw,accel_y_raw,accel_x_bias,accel_y_bias,"
            "accel_x_corrected,accel_y_corrected,beta_estimate,"
            "torque_FL,torque_FR,torque_RL,torque_RR"
        )
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label=labels, size=28, stride=28)
        ]
        msg.data = [
            float(self.phase),
            self.imu_raw,
            self.imu_bias,
            self.yaw_rate,
            self.yaw_rate_derivative,
            self.last_yaw_ref,
            self.last_yaw_error,
            self.yaw_hold / (2.0 * math.pi),
            self.odom_speed_raw,
            self.odom_speed,
            self.speed_derivative,
            self.last_speed_ref,
            self.last_speed_error,
            self.speed_integral,
            self.yaw_integral,
            self.last_speed_correction,
            self.last_yaw_correction,
            self.accel_x_raw,
            self.accel_y_raw,
            self.accel_x_bias,
            self.accel_y_bias,
            self.accel_x,
            self.accel_y,
            self.beta_estimate,
        ] + list(command)
        self.diag_pub.publish(msg)

    def open_log(self, now):
        os.makedirs(self.log_directory, exist_ok=True)
        stamp = now.nanoseconds * 1.0e-9
        filename = "drift_4wid_speed_yaw_pid_%.3f.csv" % stamp
        self.log_path = os.path.join(self.log_directory, filename)
        self.log_file = open(self.log_path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(
            [
                "time_s",
                "phase",
                "imu_raw_radps",
                "imu_bias_radps",
                "yaw_rate_filtered_radps",
                "yaw_rate_derivative_filtered_radps2",
                "yaw_ref_radps",
                "yaw_error_radps",
                "yaw_turns",
                "odom_speed_raw_mps",
                "odom_speed_filtered_mps",
                "odom_speed_derivative_filtered_mps2",
                "speed_ref_mps",
                "speed_error_mps",
                "speed_integral_m",
                "yaw_integral_rad",
                "speed_correction_nm",
                "yaw_correction_nm",
                "accel_x_raw_mps2",
                "accel_y_raw_mps2",
                "accel_x_bias_mps2",
                "accel_y_bias_mps2",
                "accel_x_corrected_mps2",
                "accel_y_corrected_mps2",
                "beta_estimate_rad",
                "beta_estimate_deg",
                "raw_FL_nm",
                "raw_FR_nm",
                "raw_RL_nm",
                "raw_RR_nm",
                "logical_FL_nm",
                "logical_FR_nm",
                "logical_RL_nm",
                "logical_RR_nm",
                "published_FL_nm",
                "published_FR_nm",
                "published_RL_nm",
                "published_RR_nm",
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
                self.last_yaw_error,
                self.yaw_hold / (2.0 * math.pi),
                self.odom_speed_raw,
                self.odom_speed,
                self.speed_derivative,
                self.last_speed_ref,
                self.last_speed_error,
                self.speed_integral,
                self.yaw_integral,
                self.last_speed_correction,
                self.last_yaw_correction,
                self.accel_x_raw,
                self.accel_y_raw,
                self.accel_x_bias,
                self.accel_y_bias,
                self.accel_x,
                self.accel_y,
                self.beta_estimate,
                math.degrees(self.beta_estimate),
            ]
            + list(self.last_raw_command)
            + list(command)
            + published
        )
        self.log_file.flush()

    @staticmethod
    def dsmoothstep(value):
        value = min(max(value, 0.0), 1.0)
        return 6.0 * value * (1.0 - value)


def main(args=None):
    rclpy.init(args=args)
    node = Drift4WidSpeedYawPid()
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
