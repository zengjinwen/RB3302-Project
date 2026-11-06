"""地图和定位；由 all.launch.py 一起启动，也可单独调试。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('rb3302_mission')) / 'config/localization.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), 'rb3302_fresh', 'student_ws', 'maps', 'rb3302_refined_58cm.yaml'])),
        LogInfo(msg=['定位地图：', LaunchConfiguration('map')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[config], remappings=[('odometry/filtered', '/odom')], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': LaunchConfiguration('map')}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[config], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
    ])
