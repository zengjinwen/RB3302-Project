"""一个终端启动底盘、传感器、地图、定位和任务。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    def include(package, filename, arguments=None):
        path = Path(get_package_share_directory(package)) / 'launch' / filename
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=(arguments or {}).items())

    return LaunchDescription([
        DeclareLaunchArgument('start', description='起点 A-X'),
        DeclareLaunchArgument('bearing', description='实际车头朝向，例如 E'),
        DeclareLaunchArgument('goals', description='四个目标，例如 B,C,B,A'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), 'rb3302_fresh', 'student_ws', 'maps', 'rb3302_refined_58cm.yaml'])),
        include('yesense_std_ros2', 'yesense_node.launch.py'),
        include('rplidar_ros', 'rplidar_launch.py'),
        include('robot_model', 'description.launch.py'),
        Node(package='rb3302_mission', executable='legacy_base', output='screen'),
        include('rb3302_mission', 'localization.launch.py', {'map': LaunchConfiguration('map')}),
        include('rb3302_mission', 'mission.launch.py',
                {key: LaunchConfiguration(key) for key in ('start', 'bearing', 'goals')}),
    ])
