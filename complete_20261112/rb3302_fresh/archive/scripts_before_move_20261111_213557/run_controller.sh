#!/usr/bin/env bash
set -e
if [ "$#" -ne 4 ]; then
    echo "用法：$0 pure_pursuit|stanley|dwa 起点 朝向 四个目标"
    exit 2
fi
case "$1" in pure_pursuit|stanley|dwa) ;; *) echo "控制器填 pure_pursuit、stanley 或 dwa"; exit 2;; esac
rb3302_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Hold the lock across exec so a second launcher cannot share the devices.
exec 9>"$rb3302_dir/.run.lock"
flock -n 9 || { echo "已有导航任务运行，请先退出旧任务。"; exit 1; }
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash
source "$rb3302_dir/robot_ws/install/setup.bash"
source "$rb3302_dir/student_ws/install/setup.bash"
exec ros2 launch rb3302_mission all.launch.py "controller:=$1" "start:=$2" "bearing:=$3" "goals:=$4"
