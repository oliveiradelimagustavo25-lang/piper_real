"""Exercise independent layers and the master entry using mock hardware only."""
import os
from pathlib import Path
import runpy
import signal
import subprocess
import time
import warnings
from contextlib import ExitStack
from types import SimpleNamespace
from functools import partial

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_prefix
from control_msgs.action import FollowJointTrajectory
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
import yaml

from my_robot_interface.action import MoveToNamed, PickPlace, SortObjects
from my_robot_interface.msg import DetectedObject, DetectedObjectArray, Failure


@pytest.fixture
def ros_probe(request, tmp_path, monkeypatch):
    """Isolated ROS node, bounded waits and cleanup of this probe's process groups."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(request.param))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                       '<MaxAutoParticipantIndex>99</MaxAutoParticipantIndex>'
                       '</Discovery></Domain></CycloneDDS>')
    def stop(process, timeout, signal_group):
        if process.poll() is None:
            if signal_group:
                os.killpg(process.pid, signal.SIGINT)
            else:
                process.send_signal(signal.SIGINT)  # launch forwards to its children.
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)

    with ExitStack() as cleanup:
        rclpy.init()
        cleanup.callback(rclpy.shutdown)
        node = rclpy.create_node('reliability_probe')
        cleanup.callback(node.destroy_node)

        def start(command, filename, timeout=5, signal_group=True):
            stream = cleanup.enter_context((tmp_path / filename).open('w'))
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            cleanup.callback(stop, process, timeout, signal_group)
            return process

        def wait(predicate, timeout=12):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    return
            pytest.fail('\n'.join(p.read_text()[-5000:] for p in tmp_path.glob('*.log')))

        def result(future, timeout=12):
            wait(future.done, timeout)
            return future.result()

        yield SimpleNamespace(node=node, start=start, wait=wait, result=result,
                              cleanup=cleanup.callback)


@pytest.mark.parametrize('combined', [False, True], ids=['independent', 'master'])
def test_mock_layers(tmp_path, monkeypatch, combined):
    monkeypatch.setenv('ROS_DOMAIN_ID', '175')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    # Humble Cyclone DDS defaults to indices 0..9; localhost-only tests need more.
    # This environment belongs to the probe and its children, never the system.
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                       '<MaxAutoParticipantIndex>99</MaxAutoParticipantIndex>'
                       '</Discovery></Domain></CycloneDDS>')
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'src/my_robot_commander/config/manipulation.yaml').read_text())
    config['pick_place_executor']['ros__parameters']['gripper_open_width'] = .04
    manipulation = tmp_path / 'manipulation.yaml'
    manipulation.write_text(yaml.safe_dump(config))
    rclpy.init()
    node = rclpy.create_node('layer_integration_probe')
    states, processes = [], []
    subscription = node.create_subscription(JointState, '/control/joint_states', states.append, 10)
    named = ActionClient(node, MoveToNamed, '/manipulation/move_to_named')
    sorting = ActionClient(node, SortObjects, '/sorting/sort_objects')
    controllers = [
        ActionClient(node, FollowJointTrajectory, f'/{group}_controller/follow_joint_trajectory')
        for group in ('arm', 'gripper')]
    release = node.create_client(Trigger, '/manipulation/release_held_object')

    def start(entry, *arguments):
        log_path = tmp_path / entry.replace('.launch.py', '.log')
        log = log_path.open('w')
        process = subprocess.Popen(
            ['ros2', 'launch', 'my_robot_bringup', entry, *arguments],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((process, log, log_path))

    def wait(predicate, timeout=25):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            assert all(p.poll() is None for p, _, _ in processes), [
                path.read_text() for _, _, path in processes]
            if predicate():
                return
        pytest.fail('Mock stack timed out: ' + '\n'.join(
            path.read_text()[-5000:] for _, _, path in processes))

    def result(future):
        wait(future.done)
        return future.result()

    visual = ('start_camera:=false', 'show_window:=false', 'publish_handeye_tf:=false')
    planning = (f'manipulation_config:={manipulation}', 'control_mode:=mock')
    try:
        if combined:
            start('robot.launch.py', *planning, *visual, 'use_rviz:=false')
        else:
            start('robot_perception.launch.py', *visual)
            wait(lambda: 'color_detector' in node.get_node_names())
            assert 'move_group' not in node.get_node_names()
            assert 'pick_place_executor' not in node.get_node_names()
            # Starting the middle before the bottom verifies dependency waiting.
            start('robot_motion_planning.launch.py', *planning)
            wait(lambda: 'task_planner' in node.get_node_names())
            assert not named.server_is_ready()
            deadline = time.monotonic() + 8.0
            wait(lambda: time.monotonic() >= deadline)  # Delayed MoveIt must not kill the executor.
            start('robot_control_moveit.launch.py', 'control_mode:=mock', 'use_rviz:=false')
        wait(lambda: states and named.server_is_ready() and release.service_is_ready())
        wait(lambda: all(client.server_is_ready() for client in controllers))
        names = node.get_node_names()
        assert {'move_group', 'robot_state_publisher', 'task_planner', 'color_detector'}.issubset(
            set(names))
        assert not any('agx_arm_ctrl' in name or 'vidu_ros2_wrapper' in name for name in names)
        assert node.count_publishers('/feedback/joint_states') == 0
        assert max(abs(value) for value in states[-1].position) < .001  # No automatic motion.
        for target in ('pose_1', 'pose_2'):
            handle = result(named.send_goal_async(MoveToNamed.Goal(target=target)))
            assert handle.accepted, target
            response = result(handle.get_result_async())
            assert response.result.success, f'{target}: {response.result.message}'
        response = result(release.call_async(Trigger.Request()))
        assert response.success, response.message
        wait(lambda: abs(dict(zip(states[-1].name, states[-1].position))['gripper'] - .04) < .001)
        # An ordinary sorting Action must start at the current pose, not force pose_1.
        wait(sorting.server_is_ready)
        arm_pose = dict(zip(states[-1].name, states[-1].position))
        handle = result(sorting.send_goal_async(SortObjects.Goal()))
        assert handle.accepted
        deadline = time.monotonic() + .6
        wait(lambda: time.monotonic() >= deadline)
        current = dict(zip(states[-1].name, states[-1].position))
        assert all(abs(current[name] - value) < .001 for name, value in arm_pose.items()
                   if name.startswith('joint'))
        result(handle.cancel_goal_async())
        assert result(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED
        # Cancel a running trajectory, then ensure a new named request is not poisoned.
        handle = result(named.send_goal_async(MoveToNamed.Goal(target='pose_1')))
        assert handle.accepted
        moving = handle.get_result_async()
        wait(lambda: any(abs(dict(zip(states[-1].name, states[-1].position))[name] - value) > .001
                         for name, value in arm_pose.items() if name.startswith('joint')))
        assert not moving.done(), 'Cancellation probe must observe an executing trajectory'
        result(handle.cancel_goal_async())
        assert result(moving).status == GoalStatus.STATUS_CANCELED
        handle = result(named.send_goal_async(MoveToNamed.Goal(target='pose_2')))
        assert result(handle.get_result_async()).result.success
    finally:
        # Stop only processes created by this test; shut down consumers first.
        cleanup_errors, known_crashes = [], []
        for process, log, path in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
                cleanup_errors.append(f'Launch did not stop: {path}')
            finally:
                log.close()
            for line in path.read_text().splitlines():
                if 'process has died' in line:
                    if '[move_group-' in line and 'exit code -11' in line:
                        known_crashes.append(path)
                    else:
                        cleanup_errors.append('Unexpected child exit: ' + line)
            if process.returncode != 0:
                cleanup_errors.append(f'Launch exited with {process.returncode}: {path}')
        named.destroy()
        sorting.destroy()
        for client in controllers:
            client.destroy()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        assert not cleanup_errors, '\n'.join(cleanup_errors)
        for path in known_crashes:
            warnings.warn('MoveIt shutdown SIGSEGV remains unresolved; see ' + str(path),
                          RuntimeWarning)


def test_control_gate_retries_and_synchronizes(tmp_path, monkeypatch):
    """Rejected acknowledgements and a restarted service must not latch the gate state."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '177')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    script = Path(get_package_prefix('agx_arm_moveit')) / 'lib/agx_arm_moveit/agx_arm_control_gate'
    control_gate = runpy.run_path(str(script))['ControlGate']
    rclpy.init(args=['--ros-args', '-p', 'gate_service_name:=probe_control_enable',
                     '-p', 'status_topics:=[probe_arm_status,probe_gripper_status]'])
    gate = control_gate()
    probe = rclpy.create_node('gate_service_probe')
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(gate)
    executor.add_node(probe)
    arm = probe.create_publisher(GoalStatusArray, 'probe_arm_status', 10)
    gripper = probe.create_publisher(GoalStatusArray, 'probe_gripper_status', 10)
    calls, state = [], {'open': False, 'reject_open': True}

    def wait(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if predicate():
                return
        pytest.fail('Control gate did not synchronize with the fake service')

    def reply(request, response):
        calls.append(request.data)
        if request.data and state.pop('delay_open', False):
            time.sleep(2.5)  # Simulate a provider applying a command after its response deadline.
        response.success = not (request.data and state['reject_open'])
        if response.success:
            state['open'] = request.data
        else:
            state['reject_open'] = False
            response.message = 'Simulated rejection'
        return response

    active = GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)])
    inactive = GoalStatusArray()
    try:
        wait(lambda: arm.get_subscription_count() > 0 and gripper.get_subscription_count() > 0)
        arm.publish(active)
        wait(lambda: gate._topic_is_active['probe_arm_status'])
        assert not calls
        service = probe.create_service(SetBool, 'probe_control_enable', reply)
        wait(lambda: gate._gate_open is True)
        assert calls[:2] == [True, True] and state['open']  # Retry the rejected request.
        gripper.publish(active)
        wait(lambda: gate._topic_is_active['probe_gripper_status'])
        arm.publish(inactive)
        wait(lambda: not gate._topic_is_active['probe_arm_status'])
        assert state['open'] and False not in calls  # Gripper is still executing.
        gripper.publish(inactive)
        wait(lambda: gate._gate_open is False)
        assert not state['open']

        probe.destroy_service(service)
        wait(lambda: gate._gate_open is None)
        state['open'] = True  # A restarted provider may have a different initial state.
        probe.create_service(SetBool, 'probe_control_enable', reply)
        wait(lambda: gate._gate_open is False)
        assert not state['open'] and calls[-1] is False

        previous = len(calls)
        state['delay_open'] = True
        arm.publish(active)
        wait(lambda: len(calls) > previous and calls[-1] is True)
        arm.publish(inactive)
        wait(lambda: not gate._topic_is_active['probe_arm_status'])
        wait(lambda: len(calls) > previous + 1 and calls[-1] is False)
        wait(lambda: gate._gate_open is False and not state['open'])
    finally:
        executor.shutdown()
        gate.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('modes,expected', [
    (['absolute'] * 3, [(-.24, -.06, .005), (-.24, -.06, .005)]),
    (['relative', 'absolute', 'absolute'], [(.12, -.06, .005), (.22, -.06, .005)]),
    (['relative'] * 3, [(.12, .12, -.115), (.22, .12, -.115)]),
    (['absolute'], None),
    (['relative', 'unknown', 'absolute'], None),
], ids=['absolute', 'mixed', 'relative', 'invalid_length', 'invalid_mode'])
@pytest.mark.parametrize('ros_probe', [179], indirect=True)
def test_planner_resolves_place_positions(tmp_path, ros_probe, modes, expected):
    """Observe real C++ planner goals; the fake executor rejects every motion request."""
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'src/my_robot_planner/config/task_planner.yaml').read_text())
    parameters = config['task_planner']['ros__parameters']
    # Even a configured between-object pose must not require a Named server for the first pick.
    parameters.update(observation_target='pose_1', observation_settle_time=0.0,
                      place_position_modes=modes, **{'red.place_position': [-.24, -.06, .005]})
    path = tmp_path / 'planner.yaml'
    path.write_text(yaml.safe_dump(config))
    binary = Path(get_package_prefix('my_robot_planner')) / 'lib/my_robot_planner/task_planner'
    command = [str(binary), '--ros-args', '--params-file', str(path)]
    if expected is None:
        response = subprocess.run(command, text=True, capture_output=True, timeout=8)
        assert response.returncode != 0
        assert 'place_position_modes' in response.stdout + response.stderr
        return

    node, wait, result = ros_probe.node, ros_probe.wait, ros_probe.result
    captured = []

    def reject_motion(goal):
        captured.append(goal)
        return GoalResponse.REJECT

    executor = ActionServer(node, PickPlace, parameters['pick_place_action'],
                            execute_callback=lambda _: PickPlace.Result(),
                            goal_callback=reject_motion)
    sorting = ActionClient(node, SortObjects, parameters['action_name'])
    ros_probe.cleanup(executor.destroy)
    ros_probe.cleanup(sorting.destroy)
    publisher = node.create_publisher(DetectedObjectArray, parameters['objects_topic'], 10)
    observations = DetectedObjectArray()
    observations.header.frame_id = 'base_link'
    obj = DetectedObject(object_id='red_cube', color=1)
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = .36, .18, -.12
    # A rotated object must still use the base frame axes for relative placement.
    obj.pose.orientation.z = obj.pose.orientation.w = 2 ** -.5
    obj.dimensions.x = obj.dimensions.y = obj.dimensions.z = .05
    observations.objects = [obj]

    def publish():
        observations.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(observations)

    node.create_timer(.05, publish)
    ros_probe.start(command, 'planner.log')
    wait(sorting.server_is_ready)
    for index, position in enumerate(expected):
        observations.objects[0].pose.position.x = .36 + .10 * index
        handle = result(sorting.send_goal_async(SortObjects.Goal(color_order=[1])))
        assert handle.accepted
        # Rejection is intentional: validate the request without executing a pick.
        assert result(handle.get_result_async()).status == GoalStatus.STATUS_ABORTED
        assert len(captured) == index + 1
        goal = captured[-1]
        assert goal.place_pose.header.frame_id == 'base_link'
        p = goal.place_pose.pose.position
        assert [p.x, p.y, p.z] == pytest.approx(position)
        assert goal.object_pose.pose.position.x == pytest.approx(.36 + .10 * index)
        q = goal.place_pose.pose.orientation
        assert [q.z, q.w] == pytest.approx([2 ** -.5, 2 ** -.5])


@pytest.mark.parametrize('ros_probe', [179], indirect=True)
def test_planner_recovers_from_observation_gap(tmp_path, ros_probe):
    """A gap beyond five seconds may recover, but expired poses cannot start a pick."""
    from rclpy.duration import Duration

    node, wait, result = ros_probe.node, ros_probe.wait, ros_probe.result
    captured = []

    def reject_motion(goal):
        captured.append(goal)
        return GoalResponse.REJECT

    server = ActionServer(node, PickPlace, '/manipulation/pick_place',
                          execute_callback=lambda _: PickPlace.Result(), goal_callback=reject_motion)
    sorting = ActionClient(node, SortObjects, '/sorting/sort_objects')
    ros_probe.cleanup(server.destroy)
    ros_probe.cleanup(sorting.destroy)
    publisher = node.create_publisher(DetectedObjectArray, '/perception/objects', 10)
    observations = DetectedObjectArray()
    observations.header.frame_id = 'base_link'
    obj = DetectedObject(object_id='red_cube', color=1)
    obj.pose.position.x, obj.pose.position.z, obj.pose.orientation.w = .3, .025, 1.
    obj.dimensions.x = obj.dimensions.y = obj.dimensions.z = .05
    fresh = False

    def publish():
        stamp = node.get_clock().now()
        if not fresh:
            stamp = stamp - Duration(seconds=10)
        observations.header.stamp = stamp.to_msg()
        publisher.publish(observations)

    node.create_timer(.05, publish)
    ros_probe.start(['ros2', 'run', 'my_robot_planner', 'task_planner'], 'observation_gap.log')
    wait(sorting.server_is_ready)
    handle = result(sorting.send_goal_async(SortObjects.Goal(color_order=[1])))
    outcome = handle.get_result_async()
    deadline = time.monotonic() + 5.3
    wait(lambda: time.monotonic() >= deadline)
    assert not outcome.done() and not captured
    observations.objects = [obj]
    deadline = time.monotonic() + .3
    wait(lambda: time.monotonic() >= deadline)
    assert not outcome.done() and not captured
    invalid = DetectedObject(object_id='invalid_observation', color=2)
    observations.objects.append(invalid)  # One invalid entry must not discard the valid red target.
    fresh = True
    assert result(outcome).status == GoalStatus.STATUS_ABORTED  # Fake executor rejects all motion.
    assert len(captured) == 1 and captured[0].object_id == obj.object_id


@pytest.mark.parametrize('scenario', [
    'color_order', 'nearest', 'all_outside', 'boundary', 'disabled', 'transformed', 'moves_outside',
    'invalid_bounds', 'invalid_length',
    'unavailable', 'execution_fault', 'recovery_required',
    'missing_color', 'missing_target', 'dependency_fault', 'unknown_facts', 'stream_lost', 'empty_observation',
])
@pytest.mark.parametrize('ros_probe', [179], indirect=True)
def test_planner_filters_grasp_region(tmp_path, ros_probe, scenario):
    """Run only the planner and synthetic detections/TF/actions; never start hardware."""
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'src/my_robot_planner/config/task_planner.yaml').read_text())
    params = config['task_planner']['ros__parameters']
    params.update(observation_target='', observation_settle_time=0.,
                  grasp_region_enabled=True,
                  grasp_region_min=[.2, -.2, -.15], grasp_region_max=[.5, .2, .1])
    failures = {'unavailable': Failure.TARGET_UNAVAILABLE,
                'execution_fault': Failure.EXECUTION_FAILED,
                'recovery_required': Failure.RECOVERY_REQUIRED,
                'dependency_fault': Failure.DEPENDENCY_UNAVAILABLE,
                'unknown_facts': Failure.TARGET_UNAVAILABLE}
    if scenario == 'disabled' or scenario in failures or scenario in (
            'missing_color', 'missing_target', 'stream_lost', 'empty_observation'):
        params['grasp_region_enabled'] = False
    params.update(observation_timeout=.6, max_observation_age=.3)
    if scenario == 'invalid_bounds':
        params['grasp_region_max'] = list(params['grasp_region_min'])
    if scenario == 'invalid_length':
        params['grasp_region_min'] = [.2]
    path = tmp_path / 'planner.yaml'
    path.write_text(yaml.safe_dump(config))
    binary = Path(get_package_prefix('my_robot_planner')) / 'lib/my_robot_planner/task_planner'
    command = [str(binary), '--ros-args', '--params-file', str(path)]
    if scenario.startswith('invalid_'):
        response = subprocess.run(command, text=True, capture_output=True, timeout=8)
        assert response.returncode != 0
        reason = 'grasp_region_min' if scenario == 'invalid_bounds' else 'Expected three finite'
        assert reason in response.stdout + response.stderr
        return
    node, result = ros_probe.node, ros_probe.result
    observations = DetectedObjectArray()
    observations.header.frame_id = 'camera' if scenario == 'transformed' else 'base_link'
    for color, x in enumerate([.7, .35, .8], start=1):
        obj = DetectedObject(object_id=f'cube_{color}', color=color)
        obj.pose.position.x, obj.pose.position.z = x, -.12
        obj.pose.orientation.w = 1.
        obj.dimensions.x = obj.dimensions.y = obj.dimensions.z = .05
        observations.objects.append(obj)
    if scenario == 'all_outside':
        observations.objects[1].pose.position.x = .8
    if scenario == 'boundary':
        for obj, point in zip(observations.objects, [(.2, -.2, -.15), (.35, 0., -.12), (.5, .2, .1)]):
            obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = point
    if scenario == 'moves_outside':
        observations.objects[0].pose.position.x = .3
    if scenario == 'missing_color':
        observations.objects = [o for o in observations.objects if o.color != 2]
    if scenario == 'empty_observation':
        observations.objects = []
    transforms = []
    for child, x in [('link6', 0.), ('camera', .5)]:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_link', child
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.transform.translation.x, tf.transform.rotation.w = x, 1.
        transforms.append(tf)
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transforms)
    if scenario == 'transformed':
        for obj in observations.objects:
            obj.pose.position.x -= .5
    captured = []

    def execute(handle):
        captured.append(handle.request.object_id)
        if scenario in failures and handle.request.object_id == 'cube_1':
            handle.abort()
            failure = Failure(code=failures[scenario], origin=Failure.EXECUTOR,
                              target_specific=scenario in ('unavailable', 'unknown_facts'),
                              command_state=Failure.TERMINAL, payload_state=Failure.CLEAR,
                              scene_state=Failure.CONSISTENT, detail='Synthetic target failure')
            if scenario == 'recovery_required':
                failure.payload_state = Failure.MAY_HOLD
            if scenario == 'unknown_facts':
                failure.command_state = failure.payload_state = failure.scene_state = Failure.UNKNOWN
            return PickPlace.Result(success=False, failure=failure,
                                    message='Synthetic target failure')
        if scenario == 'missing_target':
            observations.objects = [o for o in observations.objects if o.color != 2]
        if scenario == 'moves_outside':
            observations.objects[1].pose.position.x = .8
        handle.succeed()
        return PickPlace.Result(success=True, message='Synthetic motion completed')

    server = ActionServer(node, PickPlace, params['pick_place_action'], execute)
    sorting = ActionClient(node, SortObjects, params['action_name'])
    ros_probe.cleanup(server.destroy)
    ros_probe.cleanup(sorting.destroy)
    publisher = node.create_publisher(DetectedObjectArray, params['objects_topic'], 10)

    def publish():
        if scenario == 'stream_lost' and captured:
            return
        observations.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(observations)

    node.create_timer(.05, publish)
    ros_probe.start(command, 'region.log')
    ros_probe.wait(sorting.server_is_ready)
    handle = result(sorting.send_goal_async(SortObjects.Goal(
        strategy='nearest' if scenario == 'nearest' else 'color_order', color_order=[1, 2, 3])))
    assert handle.accepted
    outcome = result(handle.get_result_async())
    expected = {'all_outside': [], 'boundary': ['cube_1', 'cube_2', 'cube_3'],
                'disabled': ['cube_1', 'cube_2', 'cube_3'],
                'unavailable': ['cube_1', 'cube_2', 'cube_3'],
                'execution_fault': ['cube_1'], 'recovery_required': ['cube_1'],
                'missing_color': ['cube_1', 'cube_3'], 'missing_target': ['cube_1', 'cube_3'],
                'dependency_fault': ['cube_1'], 'unknown_facts': ['cube_1'],
                'stream_lost': ['cube_1'],
                'empty_observation': [],
                'moves_outside': ['cube_1']}.get(scenario, ['cube_2'])
    assert captured == expected
    partial = scenario in ('missing_color', 'missing_target', 'stream_lost', 'empty_observation')
    success = bool(expected) and scenario not in failures and not partial
    assert outcome.result.success == success
    assert outcome.status == (GoalStatus.STATUS_SUCCEEDED if success else GoalStatus.STATUS_ABORTED)
    if scenario == 'unavailable':
        assert 'completed=2' in outcome.result.message and 'unavailable cube_1' in outcome.result.message
    assert outcome.result.completed_count == len(expected) - int(scenario in failures)
    if scenario == 'all_outside':
        assert not outcome.result.failures  # Excluded objects are not failed grasps.
    if partial:
        assert outcome.result.failures
        assert outcome.result.failures[0].code == (
            Failure.OBSERVATION_STALE if scenario in ('stream_lost', 'empty_observation') else
            Failure.TARGET_UNAVAILABLE)
    if len(expected) < 3 and scenario not in failures and not partial:
        assert f'completed={len(expected)}, skipped_outside_grasp_region={3-len(expected)}' in outcome.result.message
        for color in range(1, 4):
            if f'cube_{color}' not in expected:
                assert f'cube_{color}' in outcome.result.message


@pytest.mark.parametrize('parameters,reason', [
    ({'tool_to_tcp': [0., 0., float('nan')]}, 'finite'),
    ({'lift_distance': float('nan')}, 'Invalid gripper'),
    ({'max_target_age': -1.0}, 'Invalid gripper'),
    ({'startup_timeout': .1}, 'Timed out waiting'),
])
def test_executor_rejects_bad_configuration(tmp_path, monkeypatch, parameters, reason):
    """Invalid input is rejected before any motion or device driver is started."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '184')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    config = tmp_path / 'invalid.yaml'
    config.write_text(yaml.safe_dump({'pick_place_executor': {'ros__parameters': parameters}}))
    binary = Path(get_package_prefix('my_robot_commander')) / 'lib/my_robot_commander/pick_place_executor'
    result = subprocess.run([str(binary), '--ros-args', '--params-file', str(config)],
                            text=True, capture_output=True, timeout=8)
    assert result.returncode != 0
    assert reason in result.stdout + result.stderr


@pytest.mark.parametrize('ros_probe', [186], indirect=True)
def test_executor_waits_for_missing_model(tmp_path, ros_probe):
    """A scene service alone must not start unbounded MoveGroup model loading."""
    from moveit_msgs.srv import GetPlanningScene
    node = ros_probe.node
    service = node.create_service(GetPlanningScene, '/get_planning_scene', lambda request, response: response)
    ros_probe.cleanup(lambda: node.destroy_service(service))
    path = tmp_path / 'model_wait.yaml'
    path.write_text(yaml.safe_dump({'pick_place_executor': {'ros__parameters': {'startup_timeout': 1.2}}}))
    process = ros_probe.start(['ros2', 'run', 'my_robot_commander', 'pick_place_executor',
                               '--ros-args', '--params-file', str(path)], 'model_wait.log')
    ros_probe.wait(lambda: process.poll() is not None, timeout=8)
    assert process.returncode != 0
    assert 'Timed out waiting for MoveIt scene/model' in (tmp_path / 'model_wait.log').read_text()


@pytest.mark.parametrize('initial_feedback', [False, True], ids=['no_feedback', 'measured_feedback'])
@pytest.mark.parametrize('ros_probe', [182], indirect=True)
def test_real_hardware_adapter_with_synthetic_feedback(tmp_path, ros_probe, initial_feedback):
    """Load the real adapter in ros2_control, with no SDK, CAN or real launch."""
    import xml.etree.ElementTree as ET
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from controller_manager_msgs.srv import ListControllers
    from trajectory_msgs.msg import JointTrajectoryPoint
    from control_msgs.msg import JointTolerance

    moveit = Path(get_package_share_directory('agx_arm_moveit'))
    description = xacro.process_file(str(moveit / 'config/agx_arm.urdf.xacro'), mappings={
        'arm_type': 'piper', 'effector_type': 'agx_gripper',
        'hardware_plugin': 'my_robot_hardware/AgxTopicSystem'}).toxml()
    # Use production plugin and model, with a shorter test-only goal deadline.
    model = ET.fromstring(description)
    hardware = model.find('ros2_control/hardware')
    ET.SubElement(hardware, 'param', name='startup_timeout').text = '3.0'
    config = yaml.safe_load(Path(runpy.run_path(str(moveit / 'launch/demo.launch.py'))[
        '_build_ros2_controllers_file']('piper', 'agx_gripper', 'left', '', True)).read_text())
    config['/arm_controller']['ros__parameters']['constraints']['goal_time'] = .3
    config['/gripper_controller']['ros__parameters']['constraints']['goal_time'] = .3
    config['/controller_manager']['ros__parameters']['robot_description'] = ET.tostring(
        model, encoding='unicode')
    path = tmp_path / 'hardware.yaml'
    path.write_text(yaml.safe_dump(config))
    node, wait, result = ros_probe.node, ros_probe.wait, ros_probe.result
    publisher = node.create_publisher(JointState, '/feedback/joint_states', 10)
    stop_requests = []
    def stop_hardware(request, response):
        stop_requests.append(time.monotonic())
        response.success = len(stop_requests) > 1  # First failure must not be silently accepted.
        return response
    stop_service = node.create_service(Trigger, '/control_stop', stop_hardware)
    ros_probe.cleanup(lambda: node.destroy_service(stop_service))
    states, commands = [], []
    names = [f'joint{i}' for i in range(1, 7)] + ['gripper']
    measured = [.20, 0., 0., 0., 0., 0., .025]
    mode = {'follow': False, 'valid': initial_feedback, 'paused': False}

    def receive_command(msg):
        commands.append(msg)
        if mode['follow']:
            measured[:] = [dict(zip(msg.name, msg.position))[name] for name in names]

    node.create_subscription(JointState, '/control/joint_states', receive_command, 10)
    node.create_subscription(JointState, '/controller_joint_states', states.append, 10)

    def publish():
        if mode['paused']:
            return
        msg = JointState(name=names, position=measured, velocity=[0.] * len(names))
        msg.header.stamp = node.get_clock().now().to_msg()
        if not mode['valid']:
            msg.position = list(measured)
            msg.position[0] = float('nan')
        publisher.publish(msg)

    node.create_timer(.02, publish)
    action = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    ros_probe.cleanup(action.destroy)
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')

    def goal(position):
        value = FollowJointTrajectory.Goal()
        value.trajectory.joint_names = names[:6]
        point = JointTrajectoryPoint(positions=[position, 0., 0., 0., 0., 0.])
        point.time_from_start.sec = 1
        value.trajectory.points = [point]
        handle = result(action.send_goal_async(value))
        assert handle.accepted
        return handle

    hardware_process = ros_probe.start([
               'ros2', 'run', 'controller_manager', 'ros2_control_node', '--ros-args',
               '--params-file', str(path), '-r', 'joint_states:=/controller_joint_states'], 'hardware.log')
    spawner = ros_probe.start([
               'ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster',
               'arm_controller', 'gripper_controller', '--controller-manager-timeout', '10'], 'spawner.log')
    if not initial_feedback:
        wait(lambda: hardware_process.poll() is not None)
        assert not commands
        assert 'activation refused' in (tmp_path / 'hardware.log').read_text()
        return
    wait(lambda: action.server_is_ready() and states and commands and spawner.poll() == 0)
    assert commands[0].position == pytest.approx(measured)
    mode['paused'] = True
    deadline = time.monotonic() + .65
    wait(lambda: time.monotonic() >= deadline)
    mode['paused'] = False
    wait(controllers.service_is_ready)
    response = result(controllers.call_async(ListControllers.Request()))
    assert any(c.name == 'arm_controller' and c.state == 'active' for c in response.controller)
    assert not stop_requests
    handle = goal(.25)
    assert result(handle.get_result_async()).status == GoalStatus.STATUS_ABORTED
    assert states[-1].position[states[-1].name.index('joint1')] == pytest.approx(.20)
    mode['follow'] = True
    handle = goal(.24)
    assert result(handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    # Keep a real 15 mm gap in synthetic feedback: default no longer aborts on
    # gripper width, while a caller can explicitly request the old 3 mm check.
    mode['follow'] = False
    gripper = ActionClient(node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
    ros_probe.cleanup(gripper.destroy)
    wait(gripper.server_is_ready)
    value = FollowJointTrajectory.Goal()
    value.trajectory.joint_names = ['gripper']
    value.trajectory.points = [JointTrajectoryPoint(positions=[.04])]
    value.trajectory.points[0].time_from_start.nanosec = 200000000
    for strict, status in ((False, GoalStatus.STATUS_SUCCEEDED), (True, GoalStatus.STATUS_ABORTED)):
        value.goal_tolerance = [JointTolerance(name='gripper', position=.003)] if strict else []
        handle = result(gripper.send_goal_async(value))
        assert handle.accepted
        assert result(handle.get_result_async()).status == status
        actual = states[-1].position[states[-1].name.index('gripper')]
        assert actual == pytest.approx(.025)  # Feedback must stay truthful.
    mode['follow'] = True
    handle = goal(.28)
    mode['valid'] = False  # Malformed frames must not keep hardware feedback fresh.
    assert result(handle.get_result_async()).status != GoalStatus.STATUS_SUCCEEDED
    wait(controllers.service_is_ready)
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        response = result(controllers.call_async(ListControllers.Request()))
        if all(c.state != 'active' for c in response.controller if c.name == 'arm_controller'):
            break
    else:
        pytest.fail('Controller remained active after the hardware feedback fault')
    assert 'last rejected: non-finite joint feedback' in (tmp_path / 'hardware.log').read_text()
    wait(lambda: len(stop_requests) >= 2)
    count = len(commands)
    deadline = time.monotonic() + .4
    wait(lambda: time.monotonic() >= deadline)
    assert len(commands) <= count + 2


@pytest.mark.parametrize('drop_result', [False, True, 'late'], ids=['cancel', 'lost_result', 'late_result'])
@pytest.mark.parametrize('ros_probe', [183], indirect=True)
def test_planner_execution_watchdog(tmp_path, ros_probe, drop_result):
    """A fake PickPlace never moves; simulate cancellation and a missing final result."""
    from rclpy.action import CancelResponse

    node, wait, result = ros_probe.node, ros_probe.wait, ros_probe.result
    pending = {'handle': None, 'future': None, 'cancel_seen': False}

    async def execute(handle):
        pending['handle'] = handle
        pending['future'] = rclpy.task.Future()
        await pending['future']
        handle.canceled()
        return PickPlace.Result(success=False, message='fake canceled')

    def release_cancel():
        if pending['handle'] and pending['handle'].is_cancel_requested and not drop_result:
            if not pending['future'].done():
                pending['future'].set_result(None)

    def cancel(_):
        pending['cancel_seen'] = True
        return CancelResponse.ACCEPT

    server = ActionServer(node, PickPlace, '/manipulation/pick_place', execute_callback=execute,
                          cancel_callback=cancel)
    sorting = ActionClient(node, SortObjects, '/sorting/sort_objects')
    ros_probe.cleanup(server.destroy)
    ros_probe.cleanup(sorting.destroy)
    publisher = node.create_publisher(DetectedObjectArray, '/perception/objects', 10)
    observations = DetectedObjectArray()
    observations.header.frame_id = 'base_link'
    obj = DetectedObject(object_id='red_cube', color=1)
    obj.pose.position.x, obj.pose.position.z = .3, .025
    obj.pose.orientation.w = 1.
    obj.dimensions.x = obj.dimensions.y = obj.dimensions.z = .05
    observations.objects = [obj]

    def publish():
        observations.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(observations)
        release_cancel()

    node.create_timer(.02, publish)
    path = tmp_path / 'planner.yaml'
    path.write_text(yaml.safe_dump({'task_planner': {'ros__parameters': {
        'execution_timeout': .3, 'cancel_timeout': .4, 'observation_settle_time': 0.0}}}))
    ros_probe.start(['ros2', 'run', 'my_robot_planner', 'task_planner', '--ros-args',
                     '--params-file', str(path)], 'watchdog.log')

    try:
        wait(sorting.server_is_ready)
        handle = result(sorting.send_goal_async(SortObjects.Goal(color_order=[1])))
        response = result(handle.get_result_async())
        assert response.status == GoalStatus.STATUS_ABORTED
        assert 'timed out' in response.result.message
        assert pending['cancel_seen']
        next_handle = result(sorting.send_goal_async(SortObjects.Goal(color_order=[1])))
        assert next_handle.accepted == (not drop_result)
        if drop_result == 'late':
            pending['future'].set_result(None)
            deadline = time.monotonic() + .3
            wait(lambda: time.monotonic() >= deadline)
            next_handle = result(sorting.send_goal_async(SortObjects.Goal(color_order=[1])))
            assert next_handle.accepted  # Late terminal resolves the child, never restarts the old batch.
        if next_handle.accepted:
            result(next_handle.cancel_goal_async())
            result(next_handle.get_result_async())
    finally:
        if pending['future'] and not pending['future'].done():
            pending['future'].set_result(None)


@pytest.mark.parametrize('ros_probe', [181], indirect=True)
def test_grasp_recovery_and_scene_lifecycle(tmp_path, ros_probe):
    """Fail real mock-controller actions, preserving uncertain/attached objects until recovery."""
    from controller_manager_msgs.srv import SwitchController
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene

    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'src/my_robot_commander/config/manipulation.yaml').read_text())
    config['pick_place_executor']['ros__parameters'].update(
        objects_topic='/scene_probe/objects', velocity_scaling=.5, acceleration_scaling=.5)
    path = tmp_path / 'manipulation.yaml'
    path.write_text(yaml.safe_dump(config))
    node = ros_probe.node
    wait, result = partial(ros_probe.wait, timeout=40), partial(ros_probe.result, timeout=40)
    named = ActionClient(node, MoveToNamed, '/manipulation/move_to_named')
    picking = ActionClient(node, PickPlace, '/manipulation/pick_place')
    ros_probe.cleanup(named.destroy)
    ros_probe.cleanup(picking.destroy)
    release = node.create_client(Trigger, '/manipulation/release_held_object')
    clear = node.create_client(Trigger, '/manipulation/clear_observed_objects')
    scene = node.create_client(GetPlanningScene, '/get_planning_scene')
    switch = node.create_client(SwitchController, '/controller_manager/switch_controller')
    publisher = node.create_publisher(DetectedObjectArray, '/scene_probe/objects', 10)
    widths = []

    def gripper_feedback(msg):
        if 'gripper' in msg.name:
            widths.append(msg.position[msg.name.index('gripper')])

    node.create_subscription(JointState, '/control/joint_states', gripper_feedback, 10)
    observations = DetectedObjectArray()
    observations.header.frame_id = 'base_link'

    def publish():
        observations.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(observations)

    node.create_timer(.05, publish)
    log_path = tmp_path / 'recovery.log'
    ros_probe.start([
        'ros2', 'launch', 'my_robot_bringup', 'robot.launch.py', 'control_mode:=mock',
        'start_planner:=false', 'perception_mode:=none', 'start_camera:=false',
        'show_window:=false', 'use_rviz:=false', 'publish_handeye_tf:=false',
        f'manipulation_config:={path}'], 'recovery.log', timeout=15, signal_group=False)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        wait(lambda: time.monotonic() >= deadline)

    def state():
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        value = result(scene.call_async(request)).scene
        return ({o.id for o in value.world.collision_objects},
                {o.object.id for o in value.robot_state.attached_collision_objects})

    def controller(name, activate):
        request = SwitchController.Request(strictness=2, activate_asap=True)
        request.timeout.sec = 3
        if activate:
            request.activate_controllers = [name]
        else:
            request.deactivate_controllers = [name]
        return switch.call_async(request)

    wait(lambda: named.server_is_ready() and picking.server_is_ready() and
         clear.service_is_ready() and release.service_is_ready() and switch.service_is_ready())
    # A lost observation is not proof that an obstacle has been removed.
    for number in range(2):
        obj = DetectedObject(object_id=f'probe_{number}', color=number+1)
        obj.pose.orientation.w = 1.
        obj.pose.position.x, obj.pose.position.z = 1.5, 1.5
        obj.dimensions.x = obj.dimensions.y = obj.dimensions.z = .02
        observations.objects.append(obj)
    spin(.3)
    handle = result(named.send_goal_async(MoveToNamed.Goal(target='pose_1')))
    assert result(handle.get_result_async()).result.success
    assert {'probe_0', 'probe_1', 'environment/ground'} <= state()[0]
    # Invalid updates preserve previous obstacles and do not block valid ones.
    observations.objects[0].dimensions.x = -1.
    spin(.2)
    handle = result(named.send_goal_async(MoveToNamed.Goal(target='pose_1')))
    assert result(handle.get_result_async()).result.success
    assert {'probe_0', 'probe_1'} <= state()[0]
    observations.objects = observations.objects[:1]
    spin(.2)
    handle = result(named.send_goal_async(MoveToNamed.Goal(target='pose_1')))
    assert result(handle.get_result_async()).result.success
    assert 'probe_1' in state()[0]
    observations.objects = []
    spin(.2)
    assert result(clear.call_async(Trigger.Request())).success
    assert state()[0] == {'environment/ground'}

    for stage, motor, attached in [('Move to pre-grasp', 'arm_controller', False),
                                   ('Close gripper', 'gripper_controller', False),
                                   ('Lift object', 'arm_controller', True)]:
        fault = {'future': None}

        def fail_controller(message):
            if message.feedback.message == stage and fault['future'] is None:
                fault['future'] = controller(motor, False)

        goal = PickPlace.Goal(object_id='test_cube')
        goal.object_pose.header.frame_id = goal.place_pose.header.frame_id = 'base_link'
        goal.object_pose.pose.orientation.w = goal.place_pose.pose.orientation.w = 1.
        goal.object_pose.pose.position.x, goal.object_pose.pose.position.z = .26, .015
        goal.place_pose.pose.position.x, goal.place_pose.pose.position.y = .26, .10
        goal.place_pose.pose.position.z = .015
        goal.object_dimensions.x = goal.object_dimensions.y = goal.object_dimensions.z = .03
        handle = result(picking.send_goal_async(goal, feedback_callback=fail_controller))
        assert handle.accepted
        response = result(handle.get_result_async())
        assert not response.result.success, stage
        assert response.result.failure.code == Failure.EXECUTION_FAILED
        assert response.result.failure.payload_state == (
            Failure.CLEAR if stage == 'Move to pre-grasp' else Failure.MAY_HOLD)
        assert fault['future'] is not None, response.result.message + '\n' + log_path.read_text()[-5000:]
        assert result(fault['future']).ok
        assert ('test_cube' in state()[1]) == attached
        if stage == 'Move to pre-grasp':
            assert result(controller(motor, True)).ok
            assert result(clear.call_async(Trigger.Request())).success
            continue
        # No silent empty-state reset, model clearing or new pick after a partial close/lift.
        assert not result(clear.call_async(Trigger.Request())).success
        assert not result(picking.send_goal_async(goal)).accepted
        assert result(controller(motor, True)).ok
        assert result(release.call_async(Trigger.Request())).success
        assert not state()[1]
        assert result(clear.call_async(Trigger.Request())).success
        assert state()[0] == {'environment/ground'}

    # Base-frame goals need no live camera or visible objects, including at
    # admission. The script validates hand-eye TF before starting its task.
    widths.clear()
    handle = result(picking.send_goal_async(goal))
    assert handle.accepted
    outcome = result(handle.get_result_async())
    assert outcome.result.success, outcome.result.message + '\n' + log_path.read_text()[-5000:]
    assert any(abs(w - .032) < .0002 for w in widths)  # 30 mm object + 6 mm pads - 4 mm compression.
    assert not observations.objects
    assert not state()[1]
    handle = result(named.send_goal_async(MoveToNamed.Goal(target='home')))
    assert result(handle.get_result_async()).result.success
    assert result(clear.call_async(Trigger.Request())).success

    # The real failure geometry is a local planning failure, not a control fault.
    goal.object_pose.pose.position.x, goal.object_pose.pose.position.y = .313, -.041
    goal.object_pose.pose.position.z = -.138
    goal.object_dimensions.x = goal.object_dimensions.y = goal.object_dimensions.z = .05
    handle = result(picking.send_goal_async(goal))
    response = result(handle.get_result_async())
    assert not response.result.success
    assert response.result.failure.code == Failure.PLAN_FAILED
    assert response.result.failure.target_specific
    assert response.result.failure.command_state == Failure.NOT_SENT
    assert 'Cartesian path incomplete' in response.result.message
    assert not state()[1] and result(clear.call_async(Trigger.Request())).success

    # The preferred transfer height was unreachable for this observed real target.
    goal.object_pose.pose.position.x, goal.object_pose.pose.position.y = .421, -.179
    goal.object_pose.pose.position.z = -.120
    goal.place_pose.pose.position.x, goal.place_pose.pose.position.y = .371, -.189
    goal.place_pose.pose.position.z = -.1195
    handle = result(picking.send_goal_async(goal))
    assert handle.accepted
    response = result(handle.get_result_async())
    assert response.result.success, response.result.message


@pytest.mark.parametrize('ros_probe', [185], indirect=True)
def test_motion_reconciles_late_result(tmp_path, ros_probe):
    """Actual MotionController, mock model, fake ExecuteTrajectory; no controller receives a goal."""
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import MoveItErrorCodes
    from rclpy.action import CancelResponse

    node, wait, result = ros_probe.node, ros_probe.wait, ros_probe.result
    calls, pending = [], []

    async def execute(handle):
        calls.append(handle)
        if len(calls) == 1:
            future = rclpy.task.Future()
            pending.append(future)
            await future
            handle.canceled()
        else:
            handle.succeed()
        return ExecuteTrajectory.Result(error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS))

    server = ActionServer(node, ExecuteTrajectory, '/late_execute', execute_callback=execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT)
    named = ActionClient(node, MoveToNamed, '/manipulation/move_to_named')
    ros_probe.cleanup(server.destroy)
    ros_probe.cleanup(named.destroy)
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'src/my_robot_commander/config/manipulation.yaml').read_text())
    config['pick_place_executor']['ros__parameters'].update(execution_timeout=.2, cancel_timeout=.2)
    path = tmp_path / 'motion.yaml'
    path.write_text(yaml.safe_dump(config))
    ros_probe.start(['ros2', 'launch', 'my_robot_bringup', 'robot_control_moveit.launch.py',
                     'control_mode:=mock', 'use_rviz:=false'], 'control.log',
                    timeout=15, signal_group=False)
    ros_probe.start(['ros2', 'run', 'my_robot_commander', 'pick_place_executor', '--ros-args',
                     '--params-file', str(path), *[
                         value for endpoint in ('send_goal', 'get_result', 'cancel_goal', 'status', 'feedback')
                         for value in ('-r', f'execute_trajectory/_action/{endpoint}:='
                                       f'/late_execute/_action/{endpoint}')]],
                    'motion.log', timeout=8)
    try:
        wait(named.server_is_ready, timeout=35)
        handle = result(named.send_goal_async(MoveToNamed.Goal(target='pose_1')))
        response = result(handle.get_result_async())
        assert not response.result.success
        assert response.result.failure.code == Failure.EXECUTION_RESULT_UNKNOWN
        assert len(calls) == 1
        assert not result(named.send_goal_async(MoveToNamed.Goal(target='pose_1'))).accepted
        pending[0].set_result(None)
        deadline = time.monotonic() + .3
        wait(lambda: time.monotonic() >= deadline)
        assert len(calls) == 1  # No automatic replay after the delayed terminal result.
        handle = result(named.send_goal_async(MoveToNamed.Goal(target='pose_1')))
        assert handle.accepted
        assert result(handle.get_result_async()).result.success
        assert len(calls) == 2
    finally:
        if pending and not pending[0].done():
            pending[0].set_result(None)
