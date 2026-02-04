import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # Package directories
    pkg_nav = get_package_share_directory('field_robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav, 'maps', 'field_map.yaml'),
        description='Full path to map yaml file to load')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # Create configured parameters
    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Navigation2 launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart}.items())

    # RViz2
    rviz_config_file = os.path.join(pkg_nav, 'config', 'nav2_view.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_map_yaml_cmd,
        declare_autostart_cmd,

        nav2_bringup_launch,
        rviz_cmd
    ])
