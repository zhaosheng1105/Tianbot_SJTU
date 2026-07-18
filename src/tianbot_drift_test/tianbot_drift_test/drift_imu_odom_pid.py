"""ROS 2 entry point for the drift_imu_odom_pid MATLAB port."""

from tianbot_drift_test.matlab_drift_pid import main_imu_odom


def main(args=None):
    main_imu_odom(args=args)


if __name__ == "__main__":
    main()

