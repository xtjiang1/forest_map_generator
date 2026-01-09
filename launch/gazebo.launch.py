import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    forest_map_generator_pkg_share = get_package_share_directory("forest_map_generator")

    ros_prefix = get_package_prefix("rclcpp")
    ros_lib_path = os.path.join(ros_prefix, "lib")

    world_path = PathJoinSubstitution(
        [FindPackageShare("forest_map_generator"), "worlds", "world_with_trees.world"]
    )

    model_path = os.path.join(forest_map_generator_pkg_share, "models")

    current_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    new_ign_path = (
        f"{current_ign_path}:{model_path}" if current_ign_path else model_path
    )

    gazebo_process = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "4", world_path],
        output="screen",
        additional_env={
            "IGN_GAZEBO_RESOURCE_PATH": new_ign_path,
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": ros_lib_path,
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value="world_with_trees.world",
                description="Name of the world file to launch",
            ),
            gazebo_process,
        ]
    )
