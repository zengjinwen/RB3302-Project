#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "$0")" && pwd)"
exec 9>"$root/sim/.tests.lock"
flock -n 9 || { echo '仿真测试已经运行'; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$root/robot_ws/install/setup.bash"
source "$root/student_ws/install/setup.bash"
exec python3 "$root/sim/tests/run_suite.py"
