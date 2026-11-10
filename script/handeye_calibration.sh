#!/usr/bin/env bash
# Select calibration/publishing and delegate node startup to robot_handeye.launch.py.
set -e

handeye_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
handeye_config="$handeye_workspace/config/handeye_calibration/startup.yaml"
handeye_recalibrate=false
handeye_dry_run=false
handeye_overrides=()

while (($#)); do
    case "$1" in
        --config)
            [[ $# -ge 2 ]] || { echo '--config requires a path' >&2; exit 2; }
            handeye_config="$2"; shift 2 ;;
        --recalibrate) handeye_recalibrate=true; shift ;;
        --dry-run) handeye_dry_run=true; shift ;;
        -h|--help)
            echo 'Usage: bash script/handeye_calibration.sh [--config PATH] [--recalibrate] [--dry-run] [argument:=value ...]'
            echo 'Auto: exact named .calib exists -> publish; otherwise -> calibrate.'
            echo 'Relative paths are resolved against the workspace. --dry-run only prints the command.'
            exit 0 ;;
        *:=*) handeye_overrides+=("$1"); shift ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
[[ "$handeye_config" = /* ]] || handeye_config="$handeye_workspace/$handeye_config"
handeye_ros_distro=$(/usr/bin/python3 "$handeye_workspace/script/build/handeye_calibration.py" \
    --print-ros-distro "$handeye_config")
[[ "$handeye_ros_distro" =~ ^[a-z]+$ ]] || { echo 'Invalid ROS distribution' >&2; exit 2; }
source "/opt/ros/$handeye_ros_distro/setup.bash"
if [[ ! -f "$handeye_workspace/install/local_setup.bash" ]]; then
    echo 'Build the workspace first: colcon build --packages-up-to my_robot_bringup' >&2
    exit 1
fi
source "$handeye_workspace/install/local_setup.bash"

exec /usr/bin/python3 "$handeye_workspace/script/build/handeye_calibration.py" \
    "$handeye_workspace" "$handeye_config" "$handeye_recalibrate" "$handeye_dry_run" \
    "${handeye_overrides[@]}"
