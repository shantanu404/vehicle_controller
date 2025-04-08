from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the world_file argument
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vehicle_controller'),
            'worlds',
            'demo_camera.world.xml'
        ]),
        description='Path to the world file'
    )

    # Launch the mvsim_node
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim_simulator',
        output='screen',
        parameters=[{'world_file': LaunchConfiguration('world_file')}]
    )

    return LaunchDescription([
        world_file_arg,
        mvsim_node
    ])

