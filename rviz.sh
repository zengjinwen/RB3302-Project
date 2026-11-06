#!/usr/bin/env bash
set -e
rb3302_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash
source "$rb3302_dir/robot_ws/install/setup.bash"
source "$rb3302_dir/student_ws/install/setup.bash"
# Display-only node runs on this computer; it never sends movement commands.
python3 "$rb3302_dir/student_ws/src/rb3302_mission/rb3302_mission/costmap_view.py" &
rb3302_costmap_pid=$!
trap 'kill "$rb3302_costmap_pid" 2>/dev/null || true; wait "$rb3302_costmap_pid" 2>/dev/null || true' EXIT
rviz2 -d "$rb3302_dir/student_ws/src/rb3302_mission/rviz/navigation.rviz"
