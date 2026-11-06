"""One navigation stack for simulation and hardware; only device sources differ."""
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    here = Path(__file__).parent
    root = here.parent
    sim = os.environ.get('NAV_SIM') == '1'
    config = str(here / 'nav2.yaml')
    clock = {'use_sim_time': sim}

    def include(pkg, file):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory(pkg)) / 'launch' / file)), launch_arguments={'use_sim_time': str(sim).lower()}.items())

    def script(file):
        return ExecuteProcess(cmd=['python3', str(here / file), '--ros-args', '-p', 'use_sim_time:=' + str(sim).lower()], output='screen')

    def node(pkg, exe, name, extra=None, remap=None):
        return Node(package=pkg, executable=exe, name=name, parameters=[config, clock, extra or {}], remappings=remap or [], output='screen')
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[clock, {'robot_description': (here / 'robot.urdf').read_text()}])]
    if sim:
        actions += [ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '2', os.environ['RB3302_SIM_WORLD']], output='screen'), Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': str(here / 'bridge.yaml')}]), script('sim_adapter.py')]
        if os.environ.get('SIM_HEADLESS') != '1':
            actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g', '--render-engine-gui', 'ogre', '-v', '2']))
    else:
        actions += [include('yesense_std_ros2', 'yesense_node.launch.py'), include('rplidar_ros', 'rplidar_launch.py'), script('base_driver.py')]
    actions += [node('robot_localization', 'ekf_node', 'ekf_filter_node', remap=[('odometry/filtered', '/odom')]), node('nav2_map_server', 'map_server', 'map_server', {'yaml_filename': str(here / 'map.yaml')}), node('nav2_amcl', 'amcl', 'amcl'), node('nav2_planner', 'planner_server', 'planner_server'), node('nav2_controller', 'controller_server', 'controller_server', remap=[('cmd_vel', '/cmd_vel_nav')]), node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', {'default_nav_to_pose_bt_xml': str(here / 'navigate.xml')}), node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', remap=[('cmd_vel', '/cmd_vel_nav')]), node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager', {'autostart': True, 'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'velocity_smoother', 'bt_navigator']}), script('obstacle_scan.py'), script('motion_gate.py'), script('mission.py')]
    if os.environ.get('NAV_RVIZ', '1' if sim else '0') == '1':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(here / 'navigation.rviz')], parameters=[clock]))
    return LaunchDescription(actions)
