#!/usr/bin/env bash
set -e
here="$(cd -- "$(dirname -- "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_LOCALHOST_ONLY
exec rviz2 -d "$here/navigation.rviz"
