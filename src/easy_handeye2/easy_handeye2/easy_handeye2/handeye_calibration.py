import os
import pathlib
import math
import tempfile

import yaml
from easy_handeye2_msgs.msg import HandeyeCalibration, HandeyeCalibrationParameters
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rosidl_runtime_py import set_message_fields, message_to_yaml

from . import CALIBRATIONS_DIRECTORY


def data_file(name, directory, suffix) -> pathlib.Path:
    if not name or pathlib.Path(name).name != name or name in ('.', '..'):
        raise ValueError('Calibration name must not contain directory components')
    return pathlib.Path(directory).expanduser().resolve() / f'{name}.{suffix}'


def filepath_for_calibration(name, directory=None) -> pathlib.Path:
    return data_file(name, directory or CALIBRATIONS_DIRECTORY, 'calib')


def validate_transform(transform):
    translation, rotation = transform.translation, transform.rotation
    values = (translation.x, translation.y, translation.z,
              rotation.x, rotation.y, rotation.z, rotation.w)
    norm = sum(value * value for value in values[3:])
    if not all(math.isfinite(value) for value in values) or abs(norm - 1.0) > 1e-3:
        raise ValueError('Transform must have finite values and a unit quaternion')


def write_yaml(path, message):
    """Replace a file only after its complete YAML has been written."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = pathlib.Path(stream.name)
            stream.write(message_to_yaml(message))
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class HandeyeCalibrationParametersProvider:
    def __init__(self, node: Node):
        self.node = node
        # declare and read parameters
        self.node.declare_parameter('name', '', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('calibration_type', '', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('robot_base_frame', '', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('robot_effector_frame', '', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('tracking_base_frame', '', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('tracking_marker_frame', '', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('freehand_robot_movement', True)
        self.data_directory = self.node.declare_parameter('data_directory', '').value

    def read(self):
        ret = HandeyeCalibrationParameters(
            name=self.node.get_parameter('name').get_parameter_value().string_value,
            calibration_type=self.node.get_parameter('calibration_type').get_parameter_value().string_value,
            robot_base_frame=self.node.get_parameter('robot_base_frame').get_parameter_value().string_value,
            robot_effector_frame=self.node.get_parameter('robot_effector_frame').get_parameter_value().string_value,
            tracking_base_frame=self.node.get_parameter('tracking_base_frame').get_parameter_value().string_value,
            tracking_marker_frame=self.node.get_parameter('tracking_marker_frame').get_parameter_value().string_value,
            freehand_robot_movement=self.node.get_parameter('freehand_robot_movement').get_parameter_value().bool_value,
        )
        return ret


def load_calibration(name, directory=None) -> HandeyeCalibration:
    filepath = filepath_for_calibration(name, directory)
    with open(filepath) as f:
        m = yaml.safe_load(f)
    ret = HandeyeCalibration()
    set_message_fields(ret, m)
    if ret.parameters.name != name:
        raise ValueError('Stored calibration name does not match the requested name')
    validate_transform(ret.transform)
    return ret


def save_calibration(calibration: HandeyeCalibration, directory=None) -> pathlib.Path:
    validate_transform(calibration.transform)
    filepath = filepath_for_calibration(calibration.parameters.name, directory)
    write_yaml(filepath, calibration)
    return filepath
