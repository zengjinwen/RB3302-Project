from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    root=Path(__file__).resolve().parents[1];sim=root/'sim'
    share=Path(get_package_share_directory('rb3302_mission'))
    config=str(share/'config/localization.yaml');clock={'use_sim_time':True}
    world=os.environ['RB3302_SIM_WORLD']
    flags=['-s','-r','-v','2','--headless-rendering']
    gz=ExecuteProcess(cmd=['gz','sim',*flags,world],output='screen')
    mission=ExecuteProcess(cmd=['python3',str(root/'clean/navigation.py'),'--ros-args','-p','use_sim_time:=true','-p','start:=cell_'+os.environ['SIM_START'],'-p','bearing:=dir_'+os.environ['SIM_BEARING'],'-p','goals:='+os.environ['SIM_GOALS']],output='screen')
    actions=[gz,
      Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(sim/'bridge.yaml')}],output='screen'),
      ExecuteProcess(cmd=['python3',str(sim/'adapter.py'),'--ros-args','-p','use_sim_time:=true'],output='screen'),
      IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('robot_model'))/'launch/description.launch.py')),launch_arguments={'use_sim_time':'true'}.items()),
      Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[config,clock],remappings=[('odometry/filtered','/odom')],output='screen'),
      Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[clock,{'yaml_filename':str(root/'student_ws/maps/rb3302_refined_aligned.yaml')}]),
      Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[config,clock],output='screen'),
      Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',parameters=[clock,{'autostart':True,'node_names':['map_server','amcl']}]),
      RegisterEventHandler(OnProcessExit(target_action=gz,on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
    ]
    if os.environ.get('SIM_HEADLESS')!='1':
        actions.append(ExecuteProcess(cmd=['gz','sim','-g','--render-engine-gui','ogre','-v','2'],output='screen'))
    if os.environ.get('SIM_MISSION','1')=='1':actions.append(mission)
    # Keep the scene and RViz open after a mission error so it can be inspected.
    if os.environ.get('SIM_RVIZ','1')=='1':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'clean/navigation.rviz')],parameters=[clock]))
    return LaunchDescription(actions)
