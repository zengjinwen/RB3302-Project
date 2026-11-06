#!/usr/bin/env bash
set -e
here="$(cd -- "$(dirname -- "$0")" && pwd)"
root="$(dirname -- "$here")"
if [ "$#" -ne 4 ]; then echo 'Usage: bash autonomous/run.sh sim|robot X N X,O,Q,X'; exit 2; fi
mode="$1"
export NAV_START="$2" NAV_BEARING="$3" NAV_GOALS="$4"
python3 - "$here/course.yaml" <<'CHECK'
import sys,os,yaml
points=yaml.safe_load(open(sys.argv[1]))['landmarks']
goals=os.environ['NAV_GOALS'].split(',')
if os.environ['NAV_START'] not in points or len(goals)!=4 or any(k not in points for k in goals):
    raise SystemExit('Use one valid start and exactly four comma-separated goals A-X')
if os.environ['NAV_BEARING'] not in ('N','NE','E','SE','S','SW','W','NW'):
    raise SystemExit('Bearing must be N, NE, E, SE, S, SW, W or NW')
CHECK
source /opt/ros/jazzy/setup.bash
source "$root/robot_ws/install/setup.bash"
if [ "$mode" = sim ]; then
 export NAV_SIM=1 ROS_DOMAIN_ID="${SIM_DOMAIN:-91}" ROS_LOCALHOST_ONLY=1
 export GZ_PARTITION="rb3302_autonomous_${ROS_DOMAIN_ID}"
 exec 8>"$here/.domain_${ROS_DOMAIN_ID}.lock"
 flock -n 8 || { echo 'This simulation is already running'; exit 1; }
 export RB3302_SIM_WORLD="$here/generated/world_${ROS_DOMAIN_ID}.sdf"
 args=()
 if [ -n "${SIM_OBSTACLE:-}" ]; then args+=(--obstacle "$SIM_OBSTACLE"); fi
 python3 "$root/sim/build_world.py" --root "$root" --output "$RB3302_SIM_WORLD" --start "$NAV_START" --bearing "$NAV_BEARING" "${args[@]}"
elif [ "$mode" = robot ]; then
 export NAV_SIM=0 ROS_DOMAIN_ID=7
 unset ROS_LOCALHOST_ONLY
 exec 9>"$root/.run.lock"
 flock -n 9 || { echo 'Another robot launch is running'; exit 1; }
else echo 'Mode must be sim or robot'; exit 2
fi
exec ros2 launch "$here/navigation.launch.py"
