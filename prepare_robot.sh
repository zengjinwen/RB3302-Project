#!/usr/bin/env bash
set -e
rb3302_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$rb3302_dir/robot_ws"
colcon build --symlink-install
source install/setup.bash
cd "$rb3302_dir/student_ws"
colcon build --symlink-install
source install/setup.bash
rb3302_prefix="$(ros2 pkg prefix rb3302_mission)"
for mode in mission legacy_base; do
    test -x "$rb3302_prefix/lib/rb3302_mission/$mode" || { echo "缺少可执行入口：$mode"; exit 1; }
done
python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rb3302_mission import node, robot, base_driver
import yaml
share = Path(get_package_share_directory('rb3302_mission'))
for name in ('all.launch.py', 'mission.launch.py', 'localization.launch.py'):
    assert (share/'launch'/name).is_file(), name
maps = Path.home()/'rb3302_fresh/student_ws/maps'
config = yaml.safe_load((maps/'rb3302_refined_aligned.yaml').read_text())
assert (maps/config['image']).is_file()
print('原版mission入口、Python模块、启动文件和地图检查通过。')
PY
