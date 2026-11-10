"""Exercise independent layers and the master entry using mock hardware only."""
import os
from pathlib import Path
import runpy
import signal
import subprocess
import time
import warnings

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
from my_robot_interface.msg import DetectedObject, DetectedObjectArray


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

    visual = ('start_camera:=false', 'show_window:=false')
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
def test_planner_resolves_place_positions(tmp_path, monkeypatch, modes, expected):
    """Observe real C++ planner goals; the fake executor rejects every motion request."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '179')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
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

    rclpy.init()
    node = rclpy.create_node('place_position_probe')
    captured = []

    def reject_motion(goal):
        captured.append(goal)
        return GoalResponse.REJECT

    executor = ActionServer(node, PickPlace, parameters['pick_place_action'],
                            execute_callback=lambda _: PickPlace.Result(),
                            goal_callback=reject_motion)
    sorting = ActionClient(node, SortObjects, parameters['action_name'])
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

    timer = node.create_timer(.05, publish)
    log_path = tmp_path / 'planner.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

        def wait(predicate):
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                assert process.poll() is None, log_path.read_text()
                if predicate():
                    return
            pytest.fail('Planner timed out: ' + log_path.read_text())

        def result(future):
            wait(future.done)
            return future.result()

        try:
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
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
            node.destroy_timer(timer)
            sorting.destroy()
            executor.destroy()
            node.destroy_node()
            rclpy.shutdown()


def test_grasp_recovery_and_scene_lifecycle(tmp_path, monkeypatch):
    """Fail real mock-controller actions, preserving uncertain/attached objects until recovery."""
    from controller_manager_msgs.srv import SwitchController
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene

    monkeypatch.setenv('ROS_DOMAIN_ID', '181')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                       '<MaxAutoParticipantIndex>99</MaxAutoParticipantIndex>'
                       '</Discovery></Domain></CycloneDDS>')
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'src/my_robot_commander/config/manipulation.yaml').read_text())
    config['pick_place_executor']['ros__parameters'].update(
        objects_topic='/scene_probe/objects', velocity_scaling=.5, acceleration_scaling=.5)
    path = tmp_path / 'manipulation.yaml'
    path.write_text(yaml.safe_dump(config))
    rclpy.init()
    node = rclpy.create_node('grasp_recovery_probe')
    named = ActionClient(node, MoveToNamed, '/manipulation/move_to_named')
    picking = ActionClient(node, PickPlace, '/manipulation/pick_place')
    release = node.create_client(Trigger, '/manipulation/release_held_object')
    clear = node.create_client(Trigger, '/manipulation/clear_observed_objects')
    scene = node.create_client(GetPlanningScene, '/get_planning_scene')
    switch = node.create_client(SwitchController, '/controller_manager/switch_controller')
    publisher = node.create_publisher(DetectedObjectArray, '/scene_probe/objects', 10)
    observations = DetectedObjectArray()
    observations.header.frame_id = 'base_link'

    def publish():
        observations.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(observations)

    timer = node.create_timer(.05, publish)
    log_path = tmp_path / 'recovery.log'
    log = log_path.open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'my_robot_bringup', 'robot.launch.py', 'control_mode:=mock',
        'start_planner:=false', 'perception_mode:=none', 'start_camera:=false',
        'show_window:=false', 'use_rviz:=false', f'manipulation_config:={path}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait(predicate, timeout=40):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() is None, log_path.read_text()
            if predicate():
                return
        pytest.fail(log_path.read_text()[-6000:])

    def result(future):
        wait(future.done)
        return future.result()

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

    try:
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
        observations.objects = observations.objects[:1]
        spin(.2)
        handle = result(named.send_goal_async(MoveToNamed.Goal(target='pose_1')))
        assert result(handle.get_result_async()).result.success
        assert 'probe_1' in state()[0]
        observations.objects = []
        spin(.2)
        assert result(clear.call_async(Trigger.Request())).success
        assert state()[0] == {'environment/ground'}

        for stage, motor, attached in [('Close gripper', 'gripper_controller', False),
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
            assert fault['future'] is not None, response.result.message + '\n' + log_path.read_text()[-5000:]
            assert result(fault['future']).ok
            assert ('test_cube' in state()[1]) == attached
            # No silent empty-state reset, model clearing or new pick after a partial close/lift.
            assert not result(clear.call_async(Trigger.Request())).success
            assert not result(picking.send_goal_async(goal)).accepted
            assert result(controller(motor, True)).ok
            assert result(release.call_async(Trigger.Request())).success
            assert not state()[1]
            assert result(clear.call_async(Trigger.Request())).success
            assert state()[0] == {'environment/ground'}
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
        log.close()
        node.destroy_timer(timer)
        named.destroy()
        picking.destroy()
        node.destroy_node()
        rclpy.shutdown()
