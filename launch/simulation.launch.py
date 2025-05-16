from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Declare the world_file argument
    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("vehicle_controller"),
                "worlds",
                "track00.world.xml",
            ]
        ),
        description="Path to the world file",
    )

    # Declare the headless argument
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Should run mvsim headlessly or not",
    )

    # Launch the mvsim_node
    mvsim_node = Node(
        package="mvsim",
        executable="mvsim_node",
        name="mvsim_simulator",
        output="screen",
        parameters=[
            {
                "world_file": LaunchConfiguration("world_file"),
                "headless": LaunchConfiguration("headless"),
            }
        ],
    )

    # Run rviz2 node
    rviz_conf_path = PathJoinSubstitution(
        [FindPackageShare("vehicle_controller"), "config", "demo.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_mpc_controller",
        arguments=["-d", [rviz_conf_path]],
    )

    return LaunchDescription([world_file_arg, headless_arg, mvsim_node, rviz_node])
