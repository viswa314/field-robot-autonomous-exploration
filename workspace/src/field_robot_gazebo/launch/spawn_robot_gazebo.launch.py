import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_gazebo = get_package_share_directory('field_robot_gazebo')
    pkg_description = get_package_share_directory('field_robot_description')

    # World file
    world_file = os.path.join(pkg_gazebo, 'worlds', 'field_environment.world')

    # URDF file
    urdf_file = os.path.join(pkg_description, 'urdf', 'field_robot.urdf.xacro')

    # Process xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'field_robot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot
    ])
