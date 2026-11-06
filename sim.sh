#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "$0")" && pwd)"
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
export GZ_PARTITION=rb3302_local_sim
export SIM_START="${1:-X}" SIM_BEARING="${2:-N}" SIM_GOALS="${3:-X,P,G,X}"
source /opt/ros/jazzy/setup.bash
if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
    echo '缺少Gazebo：先执行 sudo apt-get install -y ros-jazzy-ros-gz'; exit 1
fi
source "$root/robot_ws/install/setup.bash"
source "$root/student_ws/install/setup.bash"
export RB3302_SIM_WORLD="$root/sim/generated/world.sdf"
args=()
if [ -n "${SIM_OBSTACLE:-}" ]; then args+=(--obstacle "$SIM_OBSTACLE"); fi
python3 "$root/sim/build_world.py" --root "$root" --output "$RB3302_SIM_WORLD" --start "$SIM_START" --bearing "$SIM_BEARING" "${args[@]}"
exec ros2 launch "$root/sim/sim.launch.py"
