"""Hand-eye contracts and a complete synthetic-TF calibration; never opens hardware."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

from easy_handeye2.handeye_calibration import load_calibration, save_calibration
from easy_handeye2.handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
from easy_handeye2_msgs.msg import HandeyeCalibrationParameters, Sample, SampleList
from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, TakeSample
from geometry_msgs.msg import Transform, TransformStamped
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.utilities import evaluate_parameters
import numpy as np
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
import transforms3d as tfs
import yaml

ROOT = Path(__file__).resolve().parents[1]
LOGGER = SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)
ROBOT = SimpleNamespace(get_logger=lambda: LOGGER)
PARAMETERS = HandeyeCalibrationParameters(
    name='synthetic', calibration_type='eye_in_hand', robot_base_frame='test_base',
    robot_effector_frame='test_hand', tracking_base_frame='test_camera',
    tracking_marker_frame='test_marker')


def matrix(translation, angles):
    return tfs.affines.compose(translation, tfs.euler.euler2mat(*angles), [1, 1, 1])


HAND_CAMERA = matrix([.04, -.02, .08], [.2, -.1, .3])
BASE_MARKER = matrix([.5, .05, .3], [.1, .2, -.2])


def transform(pose):
    result = Transform()
    result.translation.x, result.translation.y, result.translation.z = map(float, pose[:3, 3])
    q = tfs.quaternions.mat2quat(pose[:3, :3])
    result.rotation.w, result.rotation.x, result.rotation.y, result.rotation.z = map(float, q)
    return result


def as_matrix(value):
    q, p = value.rotation, value.translation
    return tfs.affines.compose(
        [p.x, p.y, p.z], tfs.quaternions.quat2mat([q.w, q.x, q.y, q.z]), [1, 1, 1])


def poses():
    random = np.random.default_rng(42)
    return [matrix(random.uniform(-.15, .15, 3), random.uniform(-.7, .7, 3))
            for _ in range(16)]


def samples():
    return SampleList(samples=[Sample(
        robot=transform(pose), tracking=transform(np.linalg.inv(pose @ HAND_CAMERA) @ BASE_MARKER))
        for pose in poses()])


def test_solver_and_workspace_storage(tmp_path):
    result = HandeyeCalibrationBackendOpenCV().compute_calibration(ROBOT, PARAMETERS, samples())
    assert np.allclose(as_matrix(result.transform), HAND_CAMERA, atol=1e-7)
    path = save_calibration(result, tmp_path)
    assert path == tmp_path / 'synthetic.calib'
    assert np.allclose(as_matrix(load_calibration('synthetic', tmp_path).transform), HAND_CAMERA)
    content = yaml.safe_load(path.read_text())
    content['transform']['rotation']['w'] = 0.0
    path.write_text(yaml.safe_dump(content))
    with pytest.raises(ValueError, match='unit quaternion'):
        load_calibration('synthetic', tmp_path)


def test_degenerate_samples_are_rejected():
    data = samples()
    for index, sample in enumerate(data.samples):
        sample.robot = transform(matrix([index * .01, 0, 0], [0, 0, 0]))
    with pytest.raises(ValueError, match='two independent axes'):
        HandeyeCalibrationBackendOpenCV().compute_calibration(ROBOT, PARAMETERS, data)
    assert HandeyeCalibrationBackendOpenCV().compute_calibration(
        ROBOT, PARAMETERS, SampleList(samples=data.samples[:2])) is None


def test_launch_uses_existing_visual_pipeline_without_dummy_tf(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    path = ROOT / 'src/my_robot_bringup/launch/robot_handeye.launch.py'
    spec = importlib.util.spec_from_file_location('robot_handeye', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(show_gui='false', start_control='false')
    for item in module.generate_launch_description().entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(context)
    actions = module.make_nodes(context)
    assert len(actions) == 2
    assert isinstance(actions[0], IncludeLaunchDescription)
    assert actions[1].node_executable == 'handeye_server'
    context.launch_configurations['mode'] = 'publish'
    publisher, = module.make_nodes(context)
    assert publisher.node_executable == 'handeye_publisher'
    parameters = evaluate_parameters(context, publisher._Node__parameters)
    expected = yaml.safe_load((ROOT / 'config/handeye_calibration/easy_handeye2.yaml').read_text())
    assert parameters[0]['tracking_marker_frame'] == expected['/**']['ros__parameters']['tracking_marker_frame']


def test_script_auto_selection_and_overrides(tmp_path):
    config = yaml.safe_load((ROOT / 'config/handeye_calibration/startup.yaml').read_text())
    config['data_directory'] = str(tmp_path)
    startup = tmp_path / 'startup.yaml'
    startup.write_text(yaml.safe_dump(config))

    def run(*arguments):
        return subprocess.run(
            ['bash', str(ROOT / 'script/handeye_calibration.sh'), '--config', str(startup),
             '--dry-run', *arguments], cwd='/tmp', text=True, capture_output=True,
            env={**os.environ, 'ROS_DOMAIN_ID': '178'}, timeout=15)

    assert 'Mode: calibrate' in run().stdout
    (tmp_path / 'piper_vidu.samples').write_text('samples: []')
    assert 'Mode: calibrate' in run().stdout
    # Auto selects by exact filename; native publisher validates the actual content.
    (tmp_path / 'piper_vidu.calib').write_text('invalid: true')
    assert 'Mode: publish' in run().stdout
    assert 'Mode: calibrate' in run('--recalibrate').stdout
    other = run('name:=another_calibration', 'tracking_marker_frame:=aruco_marker_12')
    assert other.returncode == 0 and 'Mode: calibrate' in other.stdout
    assert 'tracking_marker_frame:=aruco_marker_12' in other.stdout
    assert run('mode:=publish', '--recalibrate').returncode != 0
    assert run('typo:=true').returncode != 0


def test_synthetic_tf_roundtrip(tmp_path, monkeypatch):
    """Use domain 178, two disconnected TF trees, native services and the saved-TF publisher."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '178')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    config = tmp_path / 'handeye.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'name': PARAMETERS.name, 'calibration_type': PARAMETERS.calibration_type,
        'robot_base_frame': PARAMETERS.robot_base_frame,
        'robot_effector_frame': PARAMETERS.robot_effector_frame,
        'tracking_base_frame': PARAMETERS.tracking_base_frame,
        'tracking_marker_frame': PARAMETERS.tracking_marker_frame,
        'minimum_samples': 15,
    }}}))
    startup = tmp_path / 'startup.yaml'
    startup_config = yaml.safe_load((ROOT / 'config/handeye_calibration/startup.yaml').read_text())
    startup_config.update(handeye_config=str(config), data_directory=str(tmp_path))
    startup_config['launch'].update(start_control=False, start_camera=False,
                                    show_gui=True, show_window=False)
    startup.write_text(yaml.safe_dump(startup_config))
    rclpy.init()
    node = rclpy.create_node('handeye_synthetic_test')
    broadcaster = TransformBroadcaster(node)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    state = {'pose': poses()[0], 'marker_visible': True}
    processes, streams = [], []

    def broadcast():
        messages = []
        pairs = [('test_base', 'test_hand', state['pose'])]
        if state['marker_visible']:
            pairs.append(('test_camera', 'test_marker',
                          np.linalg.inv(state['pose'] @ HAND_CAMERA) @ BASE_MARKER))
        for parent, child, pose in pairs:
            message = TransformStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id, message.child_frame_id = parent, child
            message.transform = transform(pose)
            messages.append(message)
        broadcaster.sendTransform(messages)

    timer = node.create_timer(.02, broadcast)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def start(mode):
        stream = (tmp_path / f'{mode}.log').open('w')
        streams.append(stream)
        process = subprocess.Popen([
            'bash', str(ROOT / 'script/handeye_calibration.sh'), '--config', str(startup),
        ], cwd='/tmp', stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)  # Let launch signal its children once.
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
                pytest.fail('Hand-eye launch did not stop on SIGINT')

    def client(service_type, suffix):
        result = node.create_client(service_type, '/easy_handeye2/calibration/' + suffix)
        end = time.monotonic() + 15
        while not result.service_is_ready() and time.monotonic() < end:
            spin(.1)
        assert result.service_is_ready(), (tmp_path / 'calibrate.log').read_text()
        return result

    def call(handle, request):
        future = handle.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done(), 'Calibration service timed out'
        return future.result()

    try:
        calibration = start('calibrate')
        take = client(TakeSample, 'take_sample')
        compute = client(ComputeCalibration, 'compute_calibration')
        save = client(SaveCalibration, 'save_calibration')
        assert not call(compute, ComputeCalibration.Request()).valid
        for index, pose in enumerate(poses(), start=1):
            state['pose'] = pose
            spin(.4)  # Hold still longer than the sampling delay.
            assert len(call(take, TakeSample.Request()).samples.samples) == index
        assert (tmp_path / 'synthetic.samples').is_file()
        result = call(compute, ComputeCalibration.Request())
        assert result.valid
        assert np.allclose(as_matrix(result.calibration.transform), HAND_CAMERA, atol=1e-5)
        state['marker_visible'] = False
        spin(.7)
        assert len(call(take, TakeSample.Request()).samples.samples) == 16
        assert not call(save, SaveCalibration.Request()).success  # Old solve invalidated.
        assert calibration.poll() is None
        assert call(compute, ComputeCalibration.Request()).valid
        saved = call(save, SaveCalibration.Request())
        assert saved.success and Path(saved.filepath.data) == tmp_path / 'synthetic.calib'
        stop(calibration)
        start('publish')
        state['marker_visible'] = True
        state['pose'] = matrix([.05, -.08, .2], [.4, -.3, .1])  # Unseen validation pose.
        spin(2.0)
        actual = buffer.lookup_transform('test_base', 'test_marker', Time(), Duration(seconds=1))
        assert np.allclose(as_matrix(actual.transform), BASE_MARKER, atol=1e-5)
    finally:
        for process in processes:
            stop(process)
        for stream in streams:
            stream.close()
        node.destroy_timer(timer)
        del listener
        node.destroy_node()
        rclpy.shutdown()
    for mode in ('calibrate', 'publish'):
        output = (tmp_path / f'{mode}.log').read_text()
        assert 'process has died' not in output and 'Traceback' not in output, output
