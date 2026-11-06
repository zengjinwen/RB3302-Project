#!/usr/bin/env bash
set -e
if [ "$#" -ne 3 ]; then
    echo "用法：$0 起点 朝向 四个目标，例如：$0 A E B,C,B,A"
    exit 2
fi
rb3302_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Hold the lock across exec so a second launcher cannot share the devices.
exec 9>"$rb3302_dir/.run.lock"
flock -n 9 || { echo "已有导航任务运行，请先退出旧任务。"; exit 1; }
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash
source "$rb3302_dir/robot_ws/install/setup.bash"
source "$rb3302_dir/student_ws/install/setup.bash"
exec ros2 launch rb3302_mission all.launch.py "start:=$1" "bearing:=$2" "goals:=$3"
