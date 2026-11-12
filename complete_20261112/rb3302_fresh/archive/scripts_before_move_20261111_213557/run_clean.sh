#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "$0")" && pwd)"
if [ "$#" -ne 3 ]; then echo '用法：bash run_clean.sh X N X,P,G,X'; exit 2; fi
exec 9>"$root/.run.lock"
flock -n 9 || { echo '旧任务仍在运行，请先Ctrl+C退出。'; exit 1; }
export ROS_DOMAIN_ID=7
export NAV_START="$1" NAV_BEARING="$2" NAV_GOALS="$3"
source /opt/ros/jazzy/setup.bash
source "$root/robot_ws/install/setup.bash"
source "$root/student_ws/install/setup.bash"
exec ros2 launch "$root/clean/robot.launch.py"
