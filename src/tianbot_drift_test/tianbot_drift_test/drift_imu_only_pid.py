"""ROS 2 entry point for the drift_imu_only_pid MATLAB port."""

from tianbot_drift_test.matlab_drift_pid import main_imu_only


def main(args=None):
    main_imu_only(args=args)


if __name__ == "__main__":
    main()

