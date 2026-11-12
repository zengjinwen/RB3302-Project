#!/usr/bin/env bash
set -e
here="$(cd -- "$(dirname -- "$0")" && pwd)"
root="$(dirname -- "$here")"
source /opt/ros/jazzy/setup.bash
missing=()
for pkg in nav2_amcl nav2_navfn_planner nav2_regulated_pure_pursuit_controller nav2_velocity_smoother nav2_bt_navigator robot_localization xacro robot_state_publisher; do
 if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then missing+=("$pkg"); fi
done
if [ "${#missing[@]}" -gt 0 ]; then
 echo "Missing ROS packages: ${missing[*]}"
 echo 'Install once: sudo apt install ros-jazzy-navigation2 ros-jazzy-robot-localization ros-jazzy-xacro ros-jazzy-robot-state-publisher python3-colcon-common-extensions python3-serial python3-yaml'
 exit 1
fi
cd "$root/robot_ws"
colcon build --symlink-install --event-handlers console_cohesion+
echo 'Ready. Future runs do not need rebuilding.'
