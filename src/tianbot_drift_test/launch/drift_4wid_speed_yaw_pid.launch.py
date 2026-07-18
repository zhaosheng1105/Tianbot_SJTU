import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("tianbot_drift_test")
    default_config = os.path.join(
        package_share, "config", "drift_4wid_speed_yaw_pid.yaml"
    )

    config = LaunchConfiguration("config")
    auto_start = LaunchConfiguration("auto_start")
    activate_mit = LaunchConfiguration("activate_mit")
    imu_yaw_sign = LaunchConfiguration("imu_yaw_sign")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("activate_mit", default_value="true"),
            DeclareLaunchArgument(
                "imu_yaw_sign",
                default_value="1.0",
                description="Right-handed Tianbot IMU: CCW positive",
            ),
            Node(
                package="tianbot_drift_test",
                executable="drift_4wid_speed_yaw_pid",
                name="drift_4wid_speed_yaw_pid",
                output="screen",
                parameters=[
                    config,
                    {
                        "auto_start": ParameterValue(
                            auto_start, value_type=bool
                        ),
                        "activate_mit_on_start": ParameterValue(
                            activate_mit, value_type=bool
                        ),
                        "imu_yaw_sign": ParameterValue(
                            imu_yaw_sign, value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
