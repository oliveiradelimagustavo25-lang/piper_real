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
handeye_ros_distro=$(/usr/bin/python3 - "$handeye_config" <<'PY'
import sys
import yaml
with open(sys.argv[1]) as stream:
    config = yaml.safe_load(stream)
print(config.get('environment', {}).get('ros_distro', 'humble'))
PY
)
[[ "$handeye_ros_distro" =~ ^[a-z]+$ ]] || { echo 'Invalid ROS distribution' >&2; exit 2; }
source "/opt/ros/$handeye_ros_distro/setup.bash"
if [[ ! -f "$handeye_workspace/install/local_setup.bash" ]]; then
    echo 'Build the workspace first: colcon build --packages-up-to my_robot_bringup' >&2
    exit 1
fi
source "$handeye_workspace/install/local_setup.bash"

exec /usr/bin/python3 - "$handeye_workspace" "$handeye_config" \
    "$handeye_recalibrate" "$handeye_dry_run" "${handeye_overrides[@]}" <<'PY'
import os
from pathlib import Path
import shlex
import sys

from ament_index_python.packages import get_package_prefix
import yaml


def main():
    workspace = Path(sys.argv[1])
    config_path = Path(sys.argv[2]).resolve()
    with config_path.open() as stream:
        config = yaml.safe_load(stream)
    overrides = dict(item.split(':=', 1) for item in sys.argv[5:])
    arguments = dict(config.get('launch', {}))
    arguments.update(handeye_config=config['handeye_config'],
                     data_directory=config['data_directory'])
    arguments.update(overrides)
    allowed = {
        'mode', 'handeye_config', 'data_directory', 'name', 'robot_base_frame',
        'robot_effector_frame', 'tracking_base_frame', 'tracking_marker_frame',
        'start_control', 'can_port', 'auto_enable', 'use_rviz', 'start_camera',
        'show_window', 'show_gui', 'use_sim_time', 'perception_config',
    }
    if arguments.keys() - allowed:
        raise ValueError(f'Unknown launch arguments: {sorted(arguments.keys() - allowed)}')
    for key in ('handeye_config', 'perception_config', 'data_directory'):
        if key in arguments:
            path = Path(arguments[key]).expanduser()
            arguments[key] = str((path if path.is_absolute() else workspace / path).resolve())
    for key in ('handeye_config', 'perception_config'):
        if key in arguments and not Path(arguments[key]).is_file():
            raise ValueError(f'{key} does not exist: {arguments[key]}')
    with open(arguments['handeye_config']) as stream:
        parameters = yaml.safe_load(stream)['/**']['ros__parameters']
    name = str(arguments.get('name') or parameters.get('name', 'piper_vidu'))
    if not name or Path(name).name != name or name in ('.', '..'):
        raise ValueError('Calibration name must be a filename without directory components')
    result = Path(arguments['data_directory']) / f'{name}.calib'
    mode = arguments.get('mode', 'auto')
    if sys.argv[3] == 'true':
        if mode not in ('auto', 'calibrate'):
            raise ValueError('--recalibrate conflicts with mode:=publish')
        mode = 'calibrate'
    if mode == 'auto':
        mode = 'publish' if result.exists() else 'calibrate'
    if mode not in ('calibrate', 'publish'):
        raise ValueError('mode must be auto, calibrate or publish')
    if mode == 'publish' and not result.is_file():
        raise ValueError(f'No calibration to publish: {result}')
    arguments['mode'] = mode

    environment = config.get('environment', {})
    os.environ.setdefault('ROS_DOMAIN_ID', str(environment.get('ros_domain_id', 189)))
    os.environ.setdefault('ROS_LOG_DIR', str(workspace / 'log/handeye'))
    for package in ('easy_handeye2', 'my_robot_bringup'):
        prefix = Path(get_package_prefix(package)).resolve()
        if not prefix.is_relative_to(workspace / 'install'):
            raise ValueError(f'{package} comes from another workspace: {prefix}; rebuild this workspace')
    command = ['ros2', 'launch', 'my_robot_bringup', 'robot_handeye.launch.py']
    command.extend(f'{key}:={str(value).lower() if isinstance(value, bool) else value}'
                   for key, value in arguments.items())
    print(f'Workspace: {workspace}\nStartup config: {config_path}\n'
          f'ROS_DOMAIN_ID: {os.environ["ROS_DOMAIN_ID"]}\nMode: {mode}\nResult: {result}')
    print(shlex.join(command), flush=True)
    if sys.argv[4] != 'true':
        os.execvpe(command[0], command, os.environ)


try:
    main()
except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
    sys.exit(f'Hand-eye startup failed: {error}')
PY
