#!/usr/bin/env python3
"""Select calibration or publishing and delegate startup to the existing launch."""
import os
from pathlib import Path
import shlex
import sys

import yaml


def main():
    if sys.argv[1:2] == ['--print-ros-distro']:
        with open(sys.argv[2]) as stream:
            config = yaml.safe_load(stream)
        print(config.get('environment', {}).get('ros_distro', 'humble'))
        return
    from ament_index_python.packages import get_package_prefix

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
