#!/usr/bin/env bash
set -e
export RB3302_MISSION=mission_corner_rpp
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run.sh" "$@"
