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
        package_share, "config", "no_steer_circle_drift_test.yaml"
    )

    config = LaunchConfiguration("config")
    use_odom_radius = LaunchConfiguration("use_odom_radius")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("use_odom_radius", default_value="false"),
            Node(
                package="tianbot_drift_test",
                executable="no_steer_circle_drift_test",
                name="no_steer_circle_drift_test",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_odom_radius_feedback": ParameterValue(
                            use_odom_radius, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
