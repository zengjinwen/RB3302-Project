import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rb3302_mission'))
    speed_parameters = ({'drive_speed': .18, 'approach_speed': .08,
                         'stanley_cross_gain': .4, 'drive_turn_limit': .50}
                        if os.environ.get('RB3302_MISSION') == 'mission_corner_rpp' else {})
    return LaunchDescription([
        DeclareLaunchArgument('start', description='Starting letter A-X'),
        DeclareLaunchArgument('bearing', description='N, NE, E, SE, S, SW, W or NW'),
        DeclareLaunchArgument('goals', description='Four ordered letters, comma separated'),
        DeclareLaunchArgument('course', default_value=str(share / 'config/course.yaml')),
        Node(package='rb3302_mission', executable=EnvironmentVariable('RB3302_MISSION', default_value='mission'), output='screen',
             parameters=[str(share / 'config/control.yaml'), speed_parameters,
                         # Prefix short enums: ROS YAML treats bare N/Y as booleans.
                         {key: ParameterValue([{'start': 'cell_', 'bearing': 'dir_'}.get(key, ''),
                                               LaunchConfiguration(key)], value_type=str)
                          for key in ('start', 'bearing', 'goals', 'course')}]),
    ])
