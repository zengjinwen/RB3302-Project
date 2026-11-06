from pathlib import Path
import shlex
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    executable = Path(get_package_prefix('rplidar_ros')) / 'lib/rplidar_ros/rplidar_node'
    script = r"""
set -e
child=""
restart_service=0
if systemctl is-active --quiet rplidar-motor.service; then restart_service=1; fi
cleanup() {
    result=$?
    trap - EXIT INT TERM
    if [ -n "$child" ] && kill -0 "$child" 2>/dev/null; then
        kill -INT "$child" 2>/dev/null || true
        for attempt in {1..20}; do
            kill -0 "$child" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$child" 2>/dev/null; then
            kill -TERM "$child" 2>/dev/null || true
            sleep 0.2
            kill -KILL "$child" 2>/dev/null || true
        fi
    fi
    if [ -n "$child" ]; then wait "$child" 2>/dev/null || true; fi
    if [ "$restart_service" -eq 1 ]; then
        sudo -n systemctl start rplidar-motor.service || true
    fi
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
sudo -n systemctl stop rplidar-motor.service
sleep 1.5
"""
    # Run the node itself: no intermediate ros2-run child to leave behind.
    script += shlex.quote(str(executable)) + """ --ros-args \
-r __node:=rplidar_node -p serial_port:=/dev/rplidar \
-p serial_baudrate:=115200 -p frame_id:=laser \
-p angle_compensate:=true -p scan_mode:=Standard &
child=$!
wait "$child"
"""
    return LaunchDescription([ExecuteProcess(cmd=['bash', '-c', script], output='screen')])
