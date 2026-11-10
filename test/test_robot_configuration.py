"""Validate model contracts and launch composition without starting hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
MOVEIT = Path(get_package_share_directory('agx_arm_moveit'))
CTRL = Path(get_package_share_directory('agx_arm_ctrl'))
BRINGUP = ROOT / 'src/my_robot_bringup/launch'


def module(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(description, **overrides):
    result = LaunchContext()
    result.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(result)
    return result


def text(ctx, substitutions):
    return perform_substitutions(ctx, normalize_to_list_of_substitutions(substitutions))


def arguments(ctx, include):
    return {name: text(ctx, value) for name, value in include.launch_arguments}


def remaps(ctx, node):
    # Inspect descriptions only; never execute a real-driver Node in this test.
    return {text(ctx, src): text(ctx, dst) for src, dst in node._Node__remappings}


@pytest.mark.parametrize('arm,effector', [
    ('piper', 'agx_gripper'), ('piper', 'none'), ('piper_x', 'agx_gripper'), ('nero', 'none')])
def test_named_states_fit_actual_model(arm, effector):
    mappings = {'arm_type': arm, 'effector_type': effector}
    urdf = ET.fromstring(xacro.process_file(
        str(MOVEIT / 'config/agx_arm.urdf.xacro'), mappings=mappings).toxml())
    srdf = ET.fromstring(xacro.process_file(
        str(MOVEIT / 'config/agx_arm.srdf.xacro'), mappings=mappings).toxml())
    joints = {joint.attrib['name']: joint for joint in urdf.findall('joint')}
    states = {state.attrib['name']: state for state in srdf.findall('group_state')}
    assert ('pose_1' in states) == (arm == 'piper')
    assert ('pose_2' in states) == (arm == 'piper')
    meshes = {mesh.attrib['filename'] for mesh in urdf.findall('.//mesh')}
    assert any(f'/agx_arm_urdf/{arm}/' in mesh for mesh in meshes)
    assert all(f'/agx_arm_urdf/{arm}/' in mesh or '/agx_arm_urdf/agx_gripper/' in mesh
               for mesh in meshes)
    for state in states.values():
        for entry in state.findall('joint'):
            limits = joints[entry.attrib['name']].find('limit')
            assert float(limits.attrib['lower']) <= float(entry.attrib['value']) <= float(
                limits.attrib['upper'])
    if effector == 'agx_gripper':
        assert 'open' not in states and 'close' not in states
        for name, width in [('gripper_open', .1), ('gripper_half', .05), ('gripper_close', 0)]:
            opening = states[name].find('joint')
            assert opening.attrib['name'] == 'gripper'
            assert float(opening.attrib['value']) == pytest.approx(width)
        for name, expected in [('gripper_joint1', .02), ('gripper_joint2', -.02)]:
            mimic = joints[name].find('mimic')
            assert mimic.attrib['joint'] == 'gripper'
            assert .04 * float(mimic.attrib['multiplier']) == pytest.approx(expected)
    assert not any('camera' in link.attrib['name'] for link in urdf.findall('link'))


@pytest.mark.parametrize('follow,expected', [
    ('true', 'feedback/joint_states'), ('false', 'control/joint_states')])
def test_follow_selects_official_state(follow, expected):
    builder = module(MOVEIT / 'launch/_moveit_config_builder.py')
    ctx = LaunchContext()
    ctx.launch_configurations['follow'] = follow
    assert builder.joint_state_topic(ctx) == expected
    names = {action.name for action in builder.declare_common_args()}
    assert not names.intersection({'feedback_topic', 'control_topic', 'enable_octomap'})


def test_official_driver_endpoints_are_not_renamed():
    driver = module(CTRL / 'launch/start_single_agx_arm.launch.py')
    desc = driver.generate_launch_description()
    ctx = context(desc)
    names = {a.name for a in desc.entities if isinstance(a, DeclareLaunchArgument)}
    assert not names.intersection({'feedback_topic', 'control_topic'})
    node = next(a for a in desc.entities if isinstance(a, Node))
    mapping = remaps(ctx, node)
    for topic in ('feedback/joint_states', 'control/joint_states'):
        assert mapping[topic] == topic


@pytest.mark.parametrize('entry', [
    'start_single_agx_arm_moveit.launch.py', 'start_single_agx_arm_rviz.launch.py'])
def test_driver_compositions_keep_official_endpoints(entry):
    launch = module(CTRL / 'launch' / entry)
    desc = launch.generate_launch_description()
    ctx = context(desc)
    names = {a.name for a in desc.entities if isinstance(a, DeclareLaunchArgument)}
    assert not names.intersection({'feedback_topic', 'control_topic'})
    for include in (a for a in desc.entities if isinstance(a, IncludeLaunchDescription)):
        args = arguments(ctx, include)
        for name, expected in [('feedback_topic', 'feedback/joint_states'),
                               ('control_topic', 'control/joint_states')]:
            if name in args:
                assert args[name] == expected


@pytest.mark.parametrize('mode,entry', [
    ('mock', 'demo.launch.py'), ('real', 'start_single_agx_arm_moveit.launch.py')])
def test_control_layer_starts_only_control(mode, entry):
    launch = module(BRINGUP / 'robot_control_moveit.launch.py')
    ctx = context(launch.generate_launch_description(), control_mode=mode)
    group, = launch.make_nodes(ctx)
    actions = group.get_sub_entities()
    include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    include.launch_description_source.get_launch_description(ctx)
    assert include.launch_description_source.location.endswith(entry)
    args = arguments(ctx, include)
    assert args['arm_type'] == 'piper' and args['effector_type'] == 'agx_gripper'
    assert args['follow'] == str(mode == 'real').lower()
    assert not any(isinstance(a, Node) for a in actions)
    if mode == 'real':
        assert args['auto_enable'] == 'false'


@pytest.mark.parametrize('mode,states', [
    ('mock', '/control/joint_states'), ('real', '/feedback/joint_states')])
def test_middle_consumes_control_without_rebuilding_model(mode, states):
    launch = module(BRINGUP / 'robot_motion_planning.launch.py')
    ctx = context(launch.generate_launch_description(), control_mode=mode)
    executor, planner = launch.make_nodes(ctx)
    assert executor.node_executable == 'pick_place_executor'
    assert planner.node_executable == 'task_planner'
    assert remaps(ctx, executor)['joint_states'] == states
    parameters = evaluate_parameters(ctx, executor._Node__parameters)
    names = {name for item in parameters if isinstance(item, dict) for name in item}
    assert any(name.startswith('robot_description_kinematics.') for name in names)
    assert 'robot_description' not in names and 'robot_description_semantic' not in names


@pytest.mark.parametrize('mode,depth,show_window,expected', [
    ('color', 'true', 'true', ['color_detector', 'depth_localizer']),
    ('color', 'false', 'false', ['color_detector']),
    ('aruco', 'true', 'true', ['aruco_detector']),
    ('both', 'true', 'true', ['color_detector', 'depth_localizer', 'aruco_detector']),
    ('both', 'false', 'false', ['color_detector', 'aruco_detector']),
    ('none', 'true', 'true', ['color_detector']),
    ('none', 'true', 'false', []),
])
def test_perception_layer_selects_nodes(mode, depth, show_window, expected):
    launch = module(BRINGUP / 'robot_perception.launch.py')
    desc = launch.generate_launch_description()
    ctx = context(desc, perception_mode=mode, enable_depth=depth,
                  show_window=show_window, start_camera='false')
    nodes = launch.make_nodes(ctx)
    assert [n.node_executable for n in nodes] == expected
    for node in nodes:
        parameters = evaluate_parameters(ctx, node._Node__parameters)
        assert Path(ctx.launch_configurations['perception_config']) in parameters
        overrides = parameters[-1]
        if node.node_executable in ('color_detector', 'aruco_detector'):
            assert overrides['show_window'] == (show_window == 'true')
        if node.node_executable == 'color_detector':
            assert overrides['enable_detection'] == (mode != 'none')
            if mode != 'none' and depth == 'false':
                assert overrides['depth_points_topic'] == ''
    ctx.launch_configurations['start_camera'] = 'true'
    assert [n.node_executable for n in launch.make_nodes(ctx)] == [
        'vidu_ros2_wrapper_node', *expected]


def test_perception_rejects_invalid_configuration():
    launch = module(BRINGUP / 'robot_perception.launch.py')
    ctx = context(launch.generate_launch_description())
    ctx.launch_configurations['perception_mode'] = 'invalid'
    with pytest.raises(ValueError, match='perception_mode'):
        launch.make_nodes(ctx)
    ctx.launch_configurations.update(perception_mode='color', perception_config='/nonexistent.yaml')
    with pytest.raises(ValueError, match='Configuration does not exist'):
        launch.make_nodes(ctx)


def test_perception_can_publish_saved_handeye_without_loading_robot():
    launch = module(BRINGUP / 'robot_perception.launch.py')
    ctx = context(launch.generate_launch_description(), publish_handeye_tf='true',
                  handeye_name='measured_mount', handeye_directory='/tmp/measured_handeye')
    *visual, publisher = launch.make_nodes(ctx)
    assert [n.node_executable for n in visual] == [
        'vidu_ros2_wrapper_node', 'color_detector', 'depth_localizer']
    args = arguments(ctx, publisher)
    assert args['mode'] == 'publish' and args['name'] == 'measured_mount'
    assert args['data_directory'] == '/tmp/measured_handeye'


@pytest.mark.parametrize('executor,planner,expected', [
    ('false', 'false', []), ('false', 'true', ['task_planner']),
    ('true', 'false', ['pick_place_executor']),
    ('true', 'true', ['pick_place_executor', 'task_planner']),
])
def test_middle_switches_load_only_enabled_components(monkeypatch, executor, planner, expected):
    launch = module(BRINGUP / 'robot_motion_planning.launch.py')
    ctx = context(launch.generate_launch_description(),
                  start_executor=executor, start_planner=planner)
    if executor == 'false':
        monkeypatch.setattr(launch.yaml, 'safe_load',
                            lambda _: pytest.fail('Disabled executor must not load kinematics'))
    assert [node.node_executable for node in launch.make_nodes(ctx)] == expected


def test_master_exposes_and_forwards_all_layer_arguments():
    launch = module(BRINGUP / 'robot.launch.py')
    desc = launch.generate_launch_description()
    ctx = context(desc, control_mode='real', perception_mode='aruco', use_sim_time='true')
    declared = [a.name for a in desc.entities if isinstance(a, DeclareLaunchArgument)]
    assert len(declared) == len(set(declared))
    groups = [a for a in desc.entities if isinstance(a, GroupAction)]
    assert len(groups) == 3
    union, shared = set(), {}
    for group in groups:
        include, = [a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription)]
        args = arguments(ctx, include)
        union.update(args)
        assert args['use_sim_time'] == 'true'
        child = include.launch_description_source.get_launch_description(ctx)
        child_declarations = [a for a in child.entities if isinstance(a, DeclareLaunchArgument)]
        child_args = {a.name for a in child_declarations}
        for action in child_declarations:
            signature = (text(ctx, action.default_value), tuple(action.choices or []))
            assert shared.setdefault(action.name, signature) == signature, action.name
        assert set(args) == child_args
        if 'control_mode' in args:
            assert args['control_mode'] == 'real'
        if 'perception_mode' in args:
            assert args['perception_mode'] == 'aruco'
    assert union == set(declared)


# Script startup contracts share this offline configuration suite.
grasp = module(ROOT / 'script/build/three_material_grasp.py')


@pytest.fixture
def valid_grasp_config(tmp_path):
    data = grasp.load_yaml(ROOT / 'config/three_material_grasp/startup.yaml')
    data.update(pre_grasp_target='pose_1', default_strategy='color_order',
                default_color_order=[1, 2, 3], place_position_modes=['absolute'] * 3)
    data['place_positions'] = {'red': [0.3, -0.1, -0.12],
                              'green': [0.3, 0.0, -0.12],
                              'blue': [0.3, 0.1, -0.12]}
    handeye = grasp.load_yaml(grasp.workspace_path(data['handeye_config']))['/**']['ros__parameters']
    directory = tmp_path / 'handeye'
    directory.mkdir()
    (directory / f'{handeye["name"]}.calib').write_text(yaml.safe_dump({
        'parameters': handeye,
        'transform': {'translation': dict(x=.04, y=.02, z=.06),
                      'rotation': dict(x=0., y=0., z=0., w=1.)},
    }))
    data['handeye_directory'] = str(directory)
    config = tmp_path / 'startup.yaml'
    config.write_text(yaml.safe_dump(data))
    return config


def test_startup_requires_calibration_and_real_place_positions(valid_grasp_config):
    config = grasp.load_settings(valid_grasp_config)
    planner = config['planner']['task_planner']['ros__parameters']
    executor = config['manipulation']['pick_place_executor']['ros__parameters']
    assert planner['red.place_position'] == [0.3, -0.1, -0.12]
    assert planner['place_position_modes'] == ['absolute'] * 3
    assert planner['one_object_per_color'] is True
    assert config['pre_grasp_target'] == planner['observation_target'] == 'pose_1'
    assert planner['default_strategy'] == config['default_strategy'] == 'color_order'
    assert planner['default_color_order'] == config['default_color_order'] == [1, 2, 3]
    assert planner['required_camera_frame'] == executor['required_camera_frame']
    assert planner['required_camera_frame'] == config['handeye']['tracking_base_frame']
    data = grasp.load_yaml(valid_grasp_config)
    data['place_positions']['red'] = None
    valid_grasp_config.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='red'):
        grasp.load_settings(valid_grasp_config)
    data['handeye_directory'] = '/nonexistent/handeye'
    valid_grasp_config.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='禁止抓取'):
        grasp.load_settings(valid_grasp_config)


def test_startup_task_parameters_override_only_the_generated_config(valid_grasp_config, tmp_path):
    settings = grasp.load_yaml(valid_grasp_config)
    base = grasp.load_yaml(grasp.workspace_path(settings['planner_config']))
    base['task_planner']['ros__parameters'].update(
        default_strategy='nearest', default_color_order=[3, 2, 1],
        place_position_modes=['relative'] * 3,
        observation_target='home', **{'red.place_position': [9., 9., 9.]})
    base_path = tmp_path / 'base_planner.yaml'
    original = yaml.safe_dump(base)
    base_path.write_text(original)
    settings.update(planner_config=str(base_path), pre_grasp_target='pose_2',
                    default_strategy='color_order', default_color_order=[2, 1, 3],
                    place_position_modes=['relative', 'absolute', 'absolute'])
    valid_grasp_config.write_text(yaml.safe_dump(settings))
    parameters = grasp.load_settings(valid_grasp_config)['planner']['task_planner']['ros__parameters']
    assert parameters['default_strategy'] == 'color_order'
    assert parameters['default_color_order'] == [2, 1, 3]
    assert parameters['observation_target'] == 'pose_2'
    assert parameters['red.place_position'] == settings['place_positions']['red']
    assert parameters['place_position_modes'] == ['relative', 'absolute', 'absolute']
    assert base_path.read_text() == original
    # Old startup files without the new field keep absolute semantics.
    del settings['place_position_modes']
    valid_grasp_config.write_text(yaml.safe_dump(settings))
    parameters = grasp.load_settings(valid_grasp_config)['planner']['task_planner']['ros__parameters']
    assert parameters['place_position_modes'] == ['absolute'] * 3
    settings['pre_grasp_target'] = ''
    valid_grasp_config.write_text(yaml.safe_dump(settings))
    config = grasp.load_settings(valid_grasp_config)
    assert config['pre_grasp_target'] == ''
    assert config['planner']['task_planner']['ros__parameters']['observation_target'] == ''


@pytest.mark.parametrize('modes', [
    ['absolute', 'relative'], ['relative', 'unknown', 'absolute'], [True, False, False]])
def test_invalid_place_position_modes_are_rejected(valid_grasp_config, modes):
    settings = grasp.load_yaml(valid_grasp_config)
    settings['place_position_modes'] = modes
    valid_grasp_config.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError, match='place_position_modes'):
        grasp.load_settings(valid_grasp_config)


@pytest.mark.parametrize('outcome', ['success', 'rejected', 'failed', 'tf_lost', 'current_pose'])
def test_optional_pre_grasp_precedes_sorting(monkeypatch, outcome):
    from concurrent.futures import Future
    from action_msgs.msg import GoalStatus
    import rclpy

    events = []
    motion_result = Future()

    def ready(value):
        future = Future()
        future.set_result(value)
        return future

    def response(success=True):
        return SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED if success else
                               GoalStatus.STATUS_ABORTED,
                               result=SimpleNamespace(success=success, message=outcome))

    class Handle:
        accepted = outcome != 'rejected'

        def get_result_async(self):
            return motion_result

        def cancel_goal_async(self):
            events.append('cancel_named')
            motion_result.set_result(response(False))
            return ready(None)

    class Named:
        def send_goal_async(self, goal, feedback_callback=None):
            assert goal.target == 'pose_1'
            events.append('send_named')
            return ready(Handle())

    class Sorting:
        def send_goal_async(self, goal, feedback_callback=None):
            if outcome == 'current_pose':
                assert not motion_result.done() and not events
            else:
                assert motion_result.done() and motion_result.result().result.success
            assert goal.strategy == '' and list(goal.color_order) == []
            assert not hasattr(goal, 'skip_initial_observation')
            events.append('send_sort')
            return ready(SimpleNamespace(accepted=True, get_result_async=lambda: ready(response())))

    def spin(*args, **kwargs):
        if not motion_result.done() and outcome != 'tf_lost':
            events.append('named_finished')
            motion_result.set_result(response(outcome != 'failed'))

    def check():
        if outcome == 'tf_lost' and 'send_named' in events:
            raise RuntimeError('camera TF lost')

    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'spin_once', spin)
    if outcome in ('success', 'current_pose'):
        target = '' if outcome == 'current_pose' else 'pose_1'
        assert grasp.execute_sequence(None, Named(), Sorting(), {'pre_grasp_target': target}, check).success
        assert events == (['send_sort'] if not target else ['send_named', 'named_finished', 'send_sort'])
    else:
        with pytest.raises(RuntimeError):
            grasp.execute_sequence(None, Named(), Sorting(), {'pre_grasp_target': 'pose_1'}, check)
        assert 'send_sort' not in events
        if outcome == 'tf_lost':
            assert 'cancel_named' in events


def test_live_tf_checks_publisher_calibration_and_freshness(valid_grasp_config):
    config = grasp.load_settings(valid_grasp_config)
    translation = SimpleNamespace(**dict(zip('xyz', config['translation'])))
    quaternion = SimpleNamespace(**dict(zip('xyzw', config['rotation'])))
    edge = SimpleNamespace(transform=SimpleNamespace(translation=translation, rotation=quaternion))
    chain = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)))

    class Buffer:
        def lookup_transform(self, target, source, stamp):
            return edge if target == config['handeye']['robot_effector_frame'] else chain

    class Node:
        publishers = [('handeye_publisher', '/')]

        def get_node_names_and_namespaces(self):
            return self.publishers

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_500_000_000))

    node = Node()
    grasp.check_tf(node, Buffer(), config)
    node.publishers = []
    with pytest.raises(RuntimeError, match='发布节点不存在'):
        grasp.check_tf(node, Buffer(), config)
    node.publishers = [('handeye_publisher', '/')]
    chain.header.stamp.sec = 95
    with pytest.raises(RuntimeError, match='过期'):
        grasp.check_tf(node, Buffer(), config)
    chain.header.stamp.sec = 100
    edge.transform.translation.x += 0.02
    with pytest.raises(RuntimeError, match='不一致'):
        grasp.check_tf(node, Buffer(), config)
