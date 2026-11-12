#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$root/robot_ws/install/setup.bash"
source "$root/student_ws/install/setup.bash"
export ROS_DOMAIN_ID=7
exec rviz2 -d "$root/clean/navigation.rviz"
