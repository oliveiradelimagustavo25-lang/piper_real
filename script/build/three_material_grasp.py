#!/usr/bin/env python3
"""Configuration and live TF guard for the existing three-layer sorting stack."""
import argparse
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import time

import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
COLORS = ('red', 'green', 'blue')


def load_yaml(path):
    with path.open() as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError(f'Expected a YAML mapping: {path}')
    return result


def workspace_path(value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else WORKSPACE / path).resolve()


def load_settings(path):
    config = load_yaml(path)
    for key, default in (('pre_grasp_target', None), ('post_grasp_target', 'home')):
        target = config.get(key, default)
        if not isinstance(target, str):
            raise ValueError(f'{key} 必须是 SRDF arm 命名姿态字符串；空字符串表示跳过该步骤')
        config[key] = target.strip()
    strategy = config.get('default_strategy')
    order = config.get('default_color_order')
    if strategy not in ('color_order', 'nearest'):
        raise ValueError('startup.yaml 的 default_strategy 必须是 color_order 或 nearest')
    if not isinstance(order, list) or any(type(v) is not int for v in order) or sorted(order) != [1, 2, 3]:
        raise ValueError('startup.yaml 的 default_color_order 必须包含 1、2、3 各一次')
    for key in ('handeye_config', 'perception_config', 'planner_config', 'manipulation_config'):
        config[key] = workspace_path(config[key])
        if not config[key].is_file():
            raise ValueError(f'{key} 文件不存在: {config[key]}')
    directory = workspace_path(config['handeye_directory'])
    handeye = load_yaml(config['handeye_config'])['/**']['ros__parameters']
    name = str(handeye['name'])
    if not name or Path(name).name != name or name in ('.', '..'):
        raise ValueError('手眼标定名称无效')
    result = directory / f'{name}.calib'
    if not result.is_file():
        raise ValueError(f'没有相机手眼标定文件，禁止抓取: {result}')
    calibration = load_yaml(result)
    params = calibration['parameters']
    for key in ('calibration_type', 'robot_base_frame', 'robot_effector_frame', 'tracking_base_frame'):
        if not handeye.get(key) or handeye[key] != params.get(key):
            raise ValueError(f'手眼标定文件与配置的 {key} 不一致')
    if params['calibration_type'] != 'eye_in_hand':
        raise ValueError('此脚本仅支持眼在手上标定')
    perception = load_yaml(config['perception_config'])
    camera = perception.get('vidu_ros2_wrapper_node', {}).get('ros__parameters', {})
    depth = perception.get('depth_localizer', {}).get('ros__parameters', {})
    if camera.get('color_frame_id', 'camera_color_optical_frame') != params['tracking_base_frame']:
        raise ValueError('视觉配置的相机坐标系与手眼标定不一致')
    if depth.get('objects_frame', 'base_link') != params['robot_base_frame']:
        raise ValueError('视觉目标坐标系与手眼标定的机器人基座不一致')
    transform = calibration['transform']
    translation = [float(transform['translation'][axis]) for axis in 'xyz']
    rotation = [float(transform['rotation'][axis]) for axis in 'xyzw']
    if not all(math.isfinite(v) for v in translation + rotation) or abs(sum(v*v for v in rotation) - 1) > 1e-3:
        raise ValueError('手眼变换含无效数值或非单位四元数')
    places = config.get('place_positions', {})
    place_modes = config.get('place_position_modes', ['absolute', 'absolute', 'absolute'])
    if not isinstance(place_modes, list) or len(place_modes) != 3 or any(
            value not in ('absolute', 'relative') for value in place_modes):
        raise ValueError('place_position_modes 必须为 x/y/z 分别指定 absolute 或 relative')
    config['place_position_modes'] = place_modes
    for color in COLORS:
        value = places.get(color)
        if not isinstance(value, list) or len(value) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
            raise ValueError(f'请先填写 {color} 的放置参数 [x, y, z]，单位 m；配合 place_position_modes 使用')
    for key in ('startup_timeout', 'max_tf_age'):
        if isinstance(config.get(key), bool) or not isinstance(config.get(key), (int, float)) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'{key} 必须为正数')
    timeouts = config.setdefault('action_timeouts', {})
    if not isinstance(timeouts, dict) or timeouts.keys() - {
            'response_timeout', 'execution_timeout', 'cancel_timeout'} or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or
            not math.isfinite(v) or v <= 0 for v in timeouts.values()):
        raise ValueError('action_timeouts 仅接受 response/execution/cancel_timeout 的有限正数')
    launch = config.get('launch', {})
    allowed = {'control_mode', 'can_port', 'auto_enable', 'use_rviz', 'show_window'}
    if launch.keys() - allowed or launch.get('control_mode') not in ('real', 'mock'):
        raise ValueError('launch 配置含未知项或 control_mode 无效')
    for key in ('auto_enable', 'use_rviz', 'show_window'):
        if not isinstance(launch.get(key), bool):
            raise ValueError(f'launch.{key} 必须是布尔值')
    planner = load_yaml(config['planner_config'])
    planner_params = planner['task_planner']['ros__parameters']
    region = config.get('grasp_region')
    if region is not None:
        if not isinstance(region, dict) or not region or region.keys() - {'enabled', 'min', 'max'}:
            raise ValueError('grasp_region 必须为 null 或仅包含 enabled/min/max 的非空映射')
        planner_params.update({f'grasp_region_{key}': value for key, value in region.items()})
    enabled = planner_params.get('grasp_region_enabled', False)
    lower, upper = (planner_params.get(f'grasp_region_{key}', [0., 0., 0.]) for key in ('min', 'max'))
    if not isinstance(enabled, bool) or any(not isinstance(v, list) or len(v) != 3 or any(
            isinstance(n, bool) or not isinstance(n, (int, float)) or not math.isfinite(n)
            for n in v) for v in (lower, upper)) or (enabled and any(a >= b for a, b in zip(lower, upper))):
        raise ValueError('grasp_region enabled 需为布尔值，min/max 需为三个有限数；启用时每轴 min < max')
    planner_params.update(grasp_region_enabled=enabled,
                          grasp_region_min=[float(v) for v in lower],
                          grasp_region_max=[float(v) for v in upper])
    if planner_params.get('reference_frame', 'base_link') != params['robot_base_frame']:
        raise ValueError('规划坐标系与手眼标定的机器人基座不一致')
    for color in COLORS:
        planner_params[f'{color}.place_position'] = [float(v) for v in places[color]]
    planner_params.update(one_object_per_color=True, observation_target=config['pre_grasp_target'],
                          default_strategy=strategy, default_color_order=order,
                          place_position_modes=place_modes)
    manipulation = load_yaml(config['manipulation_config'])
    executor_params = manipulation['pick_place_executor']['ros__parameters']
    if executor_params.get('reference_frame', 'base_link') != params['robot_base_frame']:
        raise ValueError('抓取执行器坐标系与手眼标定的机器人基座不一致')
    config.update(calibration_path=result, handeye_directory=directory, handeye=handeye,
                  translation=translation, rotation=rotation, planner=planner, manipulation=manipulation)
    return config


def make_command(config, planner_path, manipulation_path):
    args = dict(config['launch'])
    args.update(perception_mode='color', enable_depth=True, start_camera=True,
                start_executor=True, start_planner=True, publish_handeye_tf=True,
                handeye_config=config['handeye_config'], handeye_directory=config['handeye_directory'],
                perception_config=config['perception_config'], manipulation_config=manipulation_path,
                planner_config=planner_path)
    return ['ros2', 'launch', 'my_robot_bringup', 'robot.launch.py'] + [
        f'{k}:={str(v).lower() if isinstance(v, bool) else v}' for k, v in args.items()]


def check_tf(node, buffer, config):
    from rclpy.time import Time
    h = config['handeye']
    edge = buffer.lookup_transform(h['robot_effector_frame'], h['tracking_base_frame'], Time()).transform
    actual_t = [getattr(edge.translation, v) for v in 'xyz']
    actual_q = [getattr(edge.rotation, v) for v in 'xyzw']
    if (any(abs(a-b) > 1e-5 for a, b in zip(actual_t, config['translation'])) or
            abs(abs(sum(a*b for a, b in zip(actual_q, config['rotation']))) - 1.0) > 1e-4):
        raise RuntimeError('实时手眼 TF 与标定文件不一致，检查重复发布或错误坐标系')
    chain = buffer.lookup_transform(h['robot_base_frame'], h['tracking_base_frame'], Time())
    stamp = chain.header.stamp.sec + chain.header.stamp.nanosec * 1e-9
    now = node.get_clock().now().nanoseconds * 1e-9
    if stamp <= 0 or now - stamp > config['max_tf_age'] or stamp - now > 0.1:
        raise RuntimeError('机器人到相机的 TF 已过期或时间异常')


def ensure_no_stack(node):
    import rclpy
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    owned = {'controller_manager', 'move_group', 'pick_place_executor', 'task_planner',
             'vidu_ros2_wrapper_node'}
    found = sorted(name for name, namespace in node.get_node_names_and_namespaces()
                   if namespace == '/' and name in owned)
    if found:
        raise RuntimeError('当前 ROS 域已有项目节点，请先复用或关闭原栈：' + ', '.join(found))


def execute_goal(node, client, goal, check, label, feedback_callback=None,
                 response_timeout=5.0, execution_timeout=600.0, cancel_timeout=10.0):
    """Wait for one guarded action to finish before allowing the next one."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from tf2_ros import TransformException

    check()
    sent = client.send_goal_async(goal, feedback_callback=feedback_callback)
    failure = None
    response_deadline = time.monotonic() + response_timeout
    cancel_deadline = None

    def poll():
        nonlocal failure
        if not rclpy.ok():
            raise RuntimeError(f'{label}: ROS 通信已关闭，无法确认执行结果')
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
            if failure is None:
                check()
        except (TransformException, RuntimeError, KeyboardInterrupt) as error:
            if failure is None:
                failure = error

    while not sent.done():
        poll()
        if failure is None and time.monotonic() >= response_deadline:
            failure = RuntimeError('Action 请求响应超时')
        if failure is not None:
            if cancel_deadline is None:
                cancel_deadline = time.monotonic() + cancel_timeout
            if time.monotonic() >= cancel_deadline:
                raise RuntimeError(f'{label}: 未获得目标响应，执行状态未知；禁止重发，请检查控制栈')
    handle = sent.result()
    if not handle or not handle.accepted:
        raise RuntimeError(f'{label}: Action 拒绝目标')
    result = handle.get_result_async()
    canceled = False
    deadline = time.monotonic() + execution_timeout
    while not result.done():
        if failure is None and time.monotonic() >= deadline:
            failure = RuntimeError('Action 执行超时')
        if failure is not None and not canceled:
            print(f'{label}: 请求取消（{str(failure) or "用户中断"}）；'
                  '等待最终结果，软件取消不等于硬件急停。', file=sys.stderr, flush=True)
            handle.cancel_goal_async()
            canceled = True
            cancel_deadline = time.monotonic() + cancel_timeout
        if cancel_deadline is not None and time.monotonic() >= cancel_deadline:
            raise RuntimeError(f'{label}: 取消后结果未确认，执行状态未知；禁止重发，请检查控制栈')
        poll()
    if failure is not None:
        if isinstance(failure, KeyboardInterrupt):
            raise failure
        raise RuntimeError(f'{label} 因前置条件失效而终止: {failure}') from failure
    response = result.result()
    if response is None or response.result is None:
        raise RuntimeError(f'{label}: Action 返回空结果，执行状态未知')
    if response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.success:
        failures = getattr(response.result, 'failures', [])
        if not failures and hasattr(response.result, 'failure'):
            failures = [response.result.failure]
        details = '; '.join(f'code={f.code}, stage={f.stage}, object={f.object_id}: {f.detail}'
                            for f in failures if f.code != f.NONE)
        raise RuntimeError(f'{label}未完成: {response.result.message}' +
                           (f' [{details}]' if details else ''))
    return response.result


def execute_sequence(node, named_client, sorting_client, config, check):
    from my_robot_interface.action import MoveToNamed, SortObjects

    target = config['pre_grasp_target']
    timeouts = config.get('action_timeouts', {})
    if target:
        print(f'先移动到预抓取姿态: {target}', flush=True)
        execute_goal(node, named_client, MoveToNamed.Goal(target=target), check, '预定位', **timeouts)
    print('从当前位置按 startup.yaml 的默认策略和颜色顺序发送分拣任务。', flush=True)
    # Empty strategy/order select the planner defaults generated from startup.yaml.
    result = execute_goal(node, sorting_client, SortObjects.Goal(),
                          check, '分拣', feedback_callback=lambda msg: print(
                              f'当前物体: {msg.feedback.current_object_id}，'
                              f'已完成: {msg.feedback.completed_count}', flush=True), **timeouts)
    target = config.get('post_grasp_target', 'home')
    if target:
        print(f'分拣完成，回到结束姿态: {target}', flush=True)
        execute_goal(node, named_client, MoveToNamed.Goal(target=target), check, '结束回位', **timeouts)
    return result


def run(config):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.action import ActionClient
    from tf2_ros import Buffer, TransformListener, TransformException
    from my_robot_interface.action import MoveToNamed, SortObjects
    from controller_manager_msgs.srv import ListControllers
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data

    try:
        hardware = Path(get_package_prefix('my_robot_hardware'))
    except LookupError as error:
        raise RuntimeError('缺少真实反馈适配器，请先 colcon build --packages-up-to my_robot_bringup') from error
    if not (hardware / 'lib/libagx_topic_system.so').is_file():
        raise RuntimeError('真实反馈适配库未安装，请先 colcon build --packages-up-to my_robot_bringup')
    for package, executable, source in (
            ('my_robot_planner', 'task_planner', 'src/my_robot_planner/src/task_planner.cpp'),
            ('my_robot_commander', 'pick_place_executor', 'src/my_robot_commander/src/pick_place_executor.cpp')):
        prefix = Path(get_package_prefix(package)).resolve()
        binary = prefix / 'lib' / package / executable
        if not prefix.is_relative_to(WORKSPACE / 'install') or not binary.is_file() or (
                binary.stat().st_mtime_ns < (WORKSPACE / source).stat().st_mtime_ns):
            raise RuntimeError(f'{package} 未使用本工作区最新构建，请先 colcon build --packages-up-to my_robot_bringup')
    directory = Path(tempfile.mkdtemp(prefix='three_material_grasp_'))
    planner_file = directory / 'planner.yaml'
    planner_file.write_text(yaml.safe_dump(config['planner'], sort_keys=False))
    manipulation_file = directory / 'manipulation.yaml'
    manipulation_file.write_text(yaml.safe_dump(config['manipulation'], sort_keys=False))
    command = make_command(config, planner_file, manipulation_file)
    print('启动:', shlex.join(command), flush=True)
    # Retain the stack after this client exits: stopping controllers while holding is unsafe.
    rclpy.init()
    node = rclpy.create_node('three_material_grasp_client')
    try:
        ensure_no_stack(node)
        launch = subprocess.Popen(command, cwd=WORKSPACE, start_new_session=True)
    except Exception:
        node.destroy_node()
        rclpy.shutdown()
        raise
    print(f'启动进程 PID: {launch.pid}；任务结束后控制栈继续运行，需自行安全关闭。', flush=True)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    planner = config['planner']['task_planner']['ros__parameters']
    named = ActionClient(node, MoveToNamed,
                         planner.get('move_to_named_action', '/manipulation/move_to_named'))
    client = ActionClient(node, SortObjects, planner.get('action_name', '/sorting/sort_objects'))

    feedback = []
    def receive_feedback(message):
        feedback[:] = [message, time.monotonic()]
    node.create_subscription(JointState, '/feedback/joint_states', receive_feedback,
                             qos_profile_sensor_data)
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    status = None

    def check():
        if launch.poll() is not None:
            raise RuntimeError('启动进程已退出')

    try:
        deadline = time.monotonic() + config['startup_timeout']
        last_error = ''
        needs_named = bool(config['pre_grasp_target'] or config['post_grasp_target'])
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                check()
                check_tf(node, buffer, config)
                if controllers.service_is_ready() and (status is None or status.done()):
                    if status is not None:
                        active = {c.name for c in status.result().controller if c.state == 'active'}
                    else:
                        active = set()
                    status = controllers.call_async(ListControllers.Request())
                else:
                    active = set()
                fresh = config['launch'].get('control_mode', 'real') == 'mock' or (
                    bool(feedback) and time.monotonic() - feedback[1] <= 1.0 and (
                    -.1 <= node.get_clock().now().nanoseconds / 1e9 -
                    (feedback[0].header.stamp.sec + feedback[0].header.stamp.nanosec / 1e9) <= 1.0))
                if (fresh and {'arm_controller', 'gripper_controller'} <= active and
                        client.server_is_ready() and (not needs_named or named.server_is_ready())):
                    break
                last_error = '等待控制器 active、真实反馈及任务 Action 就绪'
            except (TransformException, RuntimeError) as error:
                last_error = str(error)
        else:
            raise RuntimeError(f'相机 TF 或 Action 未就绪，禁止抓取: {last_error}')
        result = execute_sequence(node, named, client, config, check)
        print(f'分拣动作完成: {result.message}；请目视核对物料。', flush=True)
    finally:
        named.destroy()
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        print(f'控制栈 PID {launch.pid} 保留运行；临时配置: {planner_file}', flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--print-ros-distro', action='store_true')
    args = parser.parse_args()
    if args.print_ros_distro:
        print(load_yaml(args.config).get('environment', {}).get('ros_distro', 'humble'))
        return
    config = load_settings(args.config)
    environment = config.get('environment', {})
    os.environ.setdefault('ROS_DOMAIN_ID', str(environment.get('ros_domain_id', 189)))
    try:
        domain = int(os.environ['ROS_DOMAIN_ID'])
    except ValueError as error:
        raise ValueError('ROS_DOMAIN_ID 必须是整数') from error
    if not 0 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_ID 必须在 0 到 232 之间')
    os.environ.setdefault('ROS_LOG_DIR', str(WORKSPACE / 'log/three_material_grasp'))
    print(f'配置检查通过。ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}')
    print('任务参数来源:', args.config.resolve())
    print('标定:', config['calibration_path'])
    print('脚本预定位/物料间观察姿态:', config['pre_grasp_target'] or '不回固定姿态，使用当前位置')
    print('脚本完成后回位姿态:', config['post_grasp_target'] or '不回位')
    print('抓取策略/颜色顺序（startup.yaml）:', config['default_strategy'], config['default_color_order'])
    print('放置轴模式 [x, y, z]（relative=原中心+偏移，absolute=绝对坐标）:', config['place_position_modes'])
    print('放置参数（startup.yaml，最终坐标由 Planner 按当前物体计算）:', config['place_positions'], flush=True)
    params = config['planner']['task_planner']['ros__parameters']
    print('抓取区域（物体中心，reference_frame 下）:',
          (params['grasp_region_min'], params['grasp_region_max'])
          if params.get('grasp_region_enabled', False) else '关闭', flush=True)
    if args.dry_run:
        print('示例命令:', shlex.join(make_command(config, '<临时 planner.yaml>', '<临时 manipulation.yaml>')))
        print('dry-run 未启动 ROS，也未检查实时 TF。')
    else:
        run(config)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, RuntimeError) as error:
        sys.exit(f'抓取流程未完成: {error}')
