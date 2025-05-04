from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Declare simulation world file option
    simulation_world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("vehicle_controller"),
                "resources",
                "worlds",
                "track00.world.xml",
            ]
        ),
        description="Path to the world file",
    )

    # Declare simulation headless argument
    simulation_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="True",
        description="Should run mvsim headlessly or not",
    )

    # Include simulation launch file
    simulation_launch_path = PathJoinSubstitution(
        [FindPackageShare("vehicle_controller"), "launch", "simulation.launch.py"]
    )

    include_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={
            "world_file": LaunchConfiguration("world_file"),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    # Declare the speed argument
    mpc_speed_arg = DeclareLaunchArgument(
        "velocity", default_value="2.5", description="Set the velocity of vehicle"
    )

    # Run pid node
    mpc_node = Node(
        package="vehicle_controller",
        executable="pid_control",
        name="pid_controller",
        output="screen",
        parameters=[{"velocity": LaunchConfiguration("velocity")}],
    )

    # Run odometry plotter
    odom_plotter_node = Node(
        package="vehicle_controller",
        executable="odometry_plotter.py",
        name="odometry_plotter",
        output="screen",
    )

    return LaunchDescription(
        [
            simulation_world_file_arg,
            simulation_headless_arg,
            include_simulation_launch,
            mpc_speed_arg,
            mpc_node,
            odom_plotter_node,
        ]
    )
