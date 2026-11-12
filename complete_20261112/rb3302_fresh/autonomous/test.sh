#!/usr/bin/env bash
set -e
here="$(cd -- "$(dirname -- "$0")" && pwd)"
exec 8>"$here/.test.lock"
flock -n 8 || { echo 'A test suite is already running'; exit 1; }
source /opt/ros/jazzy/setup.bash
exec python3 "$here/tests/run_suite.py" "$@"
