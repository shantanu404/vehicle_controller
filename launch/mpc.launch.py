from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare simulation world file option
    simulation_world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vehicle_controller'),
            'resources',
            'worlds',
            'demo_camera.world.xml'
        ]),
        description='Path to the world file'
    )

    # Declare simulation headless argument
    simulation_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Should run mvsim headlessly or not'
    )

    # Include simulation launch file
    simulation_launch_path = PathJoinSubstitution([
        FindPackageShare('vehicle_controller'),
        'launch',
        'simulation.launch.py'
    ])

    include_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    # Declare the speed argument
    mpc_speed_arg = DeclareLaunchArgument(
        'velocity',
        default_value='2.5',
        description='Set the velocity of vehicle'
    )

    # Run mpc node
    mpc_node = Node(
        package='vehicle_controller',
        executable='mpc',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'velocity' : LaunchConfiguration('velocity')
        }]
    )

    return LaunchDescription([
        simulation_world_file_arg,
        simulation_headless_arg,
        include_simulation_launch,
        mpc_speed_arg,
        mpc_node
    ])

