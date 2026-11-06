from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    def include(package,file):return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package))/'launch'/file)))
    mission=ExecuteProcess(cmd=['python3',str(root/'clean/navigation.py'),'--ros-args','-p','start:=cell_'+os.environ['NAV_START'],'-p','bearing:=dir_'+os.environ['NAV_BEARING'],'-p','goals:='+os.environ['NAV_GOALS']],output='screen')
    return LaunchDescription([include('robot_init','robot.launch.py'),include('rb3302_mission','localization.launch.py'),mission,RegisterEventHandler(OnProcessExit(target_action=mission,on_exit=[EmitEvent(event=Shutdown(reason='Navigation stopped'))]))])
