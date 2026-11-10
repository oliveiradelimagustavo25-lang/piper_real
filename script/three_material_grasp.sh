#!/usr/bin/env bash
# Start the three-layer stack and submit the existing sorting action.
set -euo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
config="$workspace/config/three_material_grasp/startup.yaml"
if [[ ${1:-} == --config ]]; then
    [[ $# -ge 2 ]] || { echo '--config needs a path' >&2; exit 2; }
    config="$2"; shift 2
fi
[[ "$config" = /* ]] || config="$workspace/$config"
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    echo 'Usage: bash script/three_material_grasp.sh [--config PATH] [--dry-run]'
    exit 0
fi
ros_distro=$(/usr/bin/python3 "$workspace/script/build/three_material_grasp.py" \
    --config "$config" --print-ros-distro)
[[ "$ros_distro" =~ ^[a-z]+$ ]] || { echo 'Invalid ROS distribution' >&2; exit 2; }
set +u  # ROS setup scripts reference optional unset variables.
source "/opt/ros/$ros_distro/setup.bash"
[[ -f "$workspace/install/local_setup.bash" ]] || { echo 'Build the workspace first.' >&2; exit 1; }
source "$workspace/install/local_setup.bash"
set -u
exec /usr/bin/python3 "$workspace/script/build/three_material_grasp.py" --config "$config" "$@"
