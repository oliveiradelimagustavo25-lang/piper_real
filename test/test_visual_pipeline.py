"""Offline ROS regression: never opens the camera or starts robot hardware."""
import copy
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import cv2
import numpy as np
import pytest
import rclpy
import yaml
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from vidu_ros2_interfaces.msg import Extrinsics

ROOT = Path(__file__).resolve().parents[1] / 'src/my_robot_perception'


def test_live_synthetic_pipeline(tmp_path):
    """Raw RGB segmentation survives depth holes; calibrated projection and display agree."""
    config = tmp_path / 'perception.yaml'
    text = (ROOT / 'config' / 'perception.yaml').read_text()
    settings = yaml.safe_load(text.replace('marker_size: 0.03', 'marker_size: 0.1'))
    settings['depth_localizer']['ros__parameters']['object_dimensions'] = [.05, .05, .05]
    config.write_text(yaml.safe_dump(settings))
    log = (tmp_path / 'nodes.log').open('w')
    rclpy.init()
    node = rclpy.create_node('offline_visual_test')
    bridge = CvBridge()
    labels, points, transforms, debug, objects = [], [], [], [], []
    from my_robot_interface.msg import DetectedObjectArray
    subscriptions = [
        node.create_subscription(Image, '/perception/color_labels', labels.append, 10),
        node.create_subscription(Image, '/perception/color_debug', debug.append, 10),
        node.create_subscription(PointCloud2, '/perception/object_points', points.append, 10),
        node.create_subscription(DetectedObjectArray, '/perception/objects', objects.append, 10),
        node.create_subscription(TFMessage, '/tf',
                                 lambda msg: transforms.extend(msg.transforms), 10)]
    raw_pub = node.create_publisher(Image, '/camera/color/origin', 10)
    depth_pub = node.create_publisher(Image, '/camera/depth', 10)
    depth_info_pub = node.create_publisher(CameraInfo, '/camera/info/depth', 10)
    color_info_pub = node.create_publisher(CameraInfo, '/camera/info/color', 10)
    extrinsics_pub = node.create_publisher(Extrinsics, '/camera/extrinsics/color_to_depth', 10)
    camera_tf = TransformStamped()
    camera_tf.header.frame_id = 'base_link'
    camera_tf.child_frame_id = 'camera_color_optical_frame'
    camera_tf.transform.translation.z = 2.0
    camera_tf.transform.rotation.x = 1.0  # Optical Z points down into the Z-up workspace.
    camera_tf.transform.rotation.w = 0.0
    static_tf = StaticTransformBroadcaster(node)
    static_tf.sendTransform(camera_tf)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def info(frame, width, height, focal):
        msg = CameraInfo()
        msg.header.frame_id = frame
        msg.width, msg.height = width, height
        msg.k = [focal, 0., width / 2., 0., focal, height / 2., 0., 0., 1.]
        msg.distortion_model, msg.d = 'plumb_bob', [0.] * 5
        return msg

    ci = info('camera_color_optical_frame', 640, 480, 500.)
    di = info('camera_depth_optical_frame', 320, 240, 250.)
    extrinsics = Extrinsics()
    extrinsics.header.frame_id = ci.header.frame_id
    extrinsics.rotation = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    extrinsics.translation = [.02, -.01, .05]
    raw = np.zeros((480, 640, 3), np.uint8)
    raw[182:233, 294:344] = (0, 0, 255)
    raw[180:230, 450:500] = (255, 0, 0)  # visible blue, no measured depth
    marker = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  7, 100)
    raw[30:170, 30:170] = 255
    raw[50:150, 50:150] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    depth = np.full((240, 320), 2.0, np.float32)  # observed background supports separation
    depth[95:116, 145:166] = 1.25
    sequence = 100

    def send(color_info=ci, depth_info=di, pixels=depth, raw_pixels=raw,
             depth_frame='camera_depth_optical_frame', with_extrinsics=True):
        nonlocal sequence
        stamps = set()
        for _ in range(7):
            sequence += 1
            stamps.add(sequence)
            for publisher, calibration in ((color_info_pub, color_info),
                                           (depth_info_pub, depth_info)):
                if calibration is not None:
                    calibration.header.stamp.sec = sequence
                    publisher.publish(calibration)
            if with_extrinsics:
                extrinsics.header.stamp.sec = sequence
                extrinsics_pub.publish(extrinsics)
            spin(.02)
            msg = bridge.cv2_to_imgmsg(raw_pixels, 'bgr8')
            msg.header.frame_id, msg.header.stamp.sec = ci.header.frame_id, sequence
            raw_pub.publish(msg)
            if pixels is not None:
                msg = bridge.cv2_to_imgmsg(
                    pixels, '32FC1' if pixels.dtype == np.float32 else '16UC1')
                msg.header.frame_id, msg.header.stamp.sec = depth_frame, sequence
                depth_pub.publish(msg)
            spin(.10)
        spin(.3)
        return stamps

    def selected(messages, stamps):
        return [m for m in messages if m.header.stamp.sec in stamps]

    def xyz(cloud):
        offsets = {f.name: f.offset for f in cloud.fields}
        return [struct.unpack_from('<f', cloud.data, offsets[k])[0] for k in ('x', 'y', 'z')]

    def check_caption(stamps, caption):
        # Compare the actual debug-image glyphs with the expected text, not GUI state.
        images = selected(debug, stamps)
        assert images
        expected = raw.copy()
        cv2.putText(expected, caption, (294, 177), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0, 0, 255), 1)
        roi = expected[163:180, 294:435]
        assert any(np.array_equal(bridge.imgmsg_to_cv2(m, 'bgr8')[163:180, 294:435], roi)
                   for m in images)

    process = subprocess.Popen([
        'ros2', 'launch', str(ROOT.parent / 'my_robot_bringup/launch/robot_perception.launch.py'),
        f'perception_config:={config}', 'perception_mode:=both',
        'start_camera:=false', 'show_window:=false', 'publish_handeye_tf:=false'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline and (
                raw_pub.get_subscription_count() < 2 or depth_pub.get_subscription_count() < 1):
            assert process.poll() is None, (tmp_path / 'nodes.log').read_text()
            spin(.1)
        assert raw_pub.get_subscription_count() == 2
        assert depth_pub.get_subscription_count() == 1
        stamps = send(None, None, pixels=None, with_extrinsics=False)
        assert selected(labels, stamps)  # no calibration/depth cannot suppress RGB detection
        check_caption(stamps, 'red1: -- mm')
        stamps = send()
        clouds = selected(points, stamps)
        cloud = next(m for m in reversed(clouds) if m.width)
        assert cloud.header.frame_id == ci.header.frame_id and cloud.width == 1
        assert xyz(cloud) == pytest.approx([-.005, -.085, 1.30], abs=.002)
        label = bridge.imgmsg_to_cv2(selected(labels, stamps)[-1])
        assert label[210, 310] == 1 and label[210, 460] == 3
        check_caption(stamps, 'red1: 1300 mm')
        full_centres = np.array([[obj.pose.position.x, obj.pose.position.y]
                                for message in selected(objects, stamps)
                                for obj in message.objects if obj.color == 1])
        assert len(full_centres)
        poses = [t for t in selected(transforms, stamps) if t.child_frame_id == 'aruco_marker_7']
        assert poses and poses[-1].transform.translation.z == pytest.approx(.5, abs=.015)
        assert poses[-1].transform.translation.x == pytest.approx(-.22, abs=.004)

        # Symmetric depth noise must not become a high object centre through
        # selecting the upper tail of the measured surface heights.
        for noise in (np.random.default_rng(17).uniform(-.004, .004, (21, 21)),
                      np.where(np.indices((21, 21)).sum(axis=0) % 2, .003, -.003)):
            noisy = depth.copy()
            noisy[95:116, 145:166] += noise
            stamps = send(pixels=noisy)
            centres = [obj.pose.position.z for message in selected(objects, stamps)
                       for obj in message.objects if obj.color == 1]
            assert centres and np.median(centres) == pytest.approx(.675, abs=.0015)

        partial = depth.copy()
        partial[95:116, 155:166] = np.nan
        stamps = send(pixels=partial)
        assert np.array_equal(bridge.imgmsg_to_cv2(selected(labels, stamps)[-1]), label)
        assert any(m.width == 1 for m in selected(points, stamps))
        partial_centres = np.array([[obj.pose.position.x, obj.pose.position.y]
                                   for message in selected(objects, stamps)
                                   for obj in message.objects if obj.color == 1])
        assert len(partial_centres)
        assert np.median(partial_centres, axis=0) == pytest.approx(
            np.median(full_centres, axis=0), abs=.002)
        check_caption(stamps, 'red1: 1300 mm')
        # Depth loss retains the full RGB regions and clears the numeric annotation.
        stamps = send(pixels=np.full_like(depth, np.nan))
        assert np.array_equal(bridge.imgmsg_to_cv2(selected(labels, stamps)[-1]), label)
        assert all(m.width == 0 for m in selected(points, stamps))
        check_caption(stamps, 'red1: -- mm')
        stamps = send(pixels=None)
        assert selected(labels, stamps)
        check_caption(stamps, 'red1: -- mm')
        # An older frame's result must not label a new frame with its depth.
        stamps = send(depth_frame='wrong_optical_frame')
        assert selected(labels, stamps)
        assert all(m.width == 0 for m in selected(points, stamps))
        check_caption(stamps, 'red1: -- mm')
        stamps = send(pixels=np.where(np.isfinite(depth), depth * 1000, 0).astype(np.uint16))
        cloud = next(m for m in reversed(selected(points, stamps)) if m.width)
        assert xyz(cloud) == pytest.approx([-.005, -.085, 1.30], abs=.002)
        # A real rotation plus translation distinguishes the SDK direction from its inverse.
        rotation, _ = cv2.Rodrigues(np.array([.02, -.03, .04]))
        extrinsics.rotation = rotation.astype(np.float32).reshape(-1).tolist()
        expected = rotation @ np.array([-.025, -.075, 1.25]) + np.array([.02, -.01, .05])
        rotated_raw = raw.copy()
        rotated_raw[170:240, 270:350] = (0, 0, 255)
        stamps = send(raw_pixels=rotated_raw)
        cloud = next(m for m in reversed(selected(points, stamps)) if m.width)
        assert xyz(cloud) == pytest.approx(expected, abs=.004)
        extrinsics.rotation = [1., 0., 0., 0., 1., 0., 0., 0., 1.]

        # With no measured surround, depth association is unverified, not fabricated.
        isolated = depth.copy()
        isolated[isolated == 2.0] = np.nan
        stamps = send(pixels=isolated)
        assert all(m.width == 0 for m in selected(points, stamps))
        assert np.array_equal(bridge.imgmsg_to_cv2(selected(labels, stamps)[-1]), label)
        check_caption(stamps, 'red1: -- mm')

        # ToF sees the background through an RGB-only occlusion. Foreground
        # rays are all missing; a background depth must not be labeled as the object.
        yy, xx = np.indices(depth.shape)

        def foreground_rays(rgb_z):
            z = rgb_z - .05
            u = 500 * (((xx - 160) / 250) * z + .02) / rgb_z + 320
            v = 500 * (((yy - 120) / 250) * z - .01) / rgb_z + 240
            return (u >= 294) & (u < 344) & (v >= 182) & (v < 233)

        background = np.ones_like(depth)
        background[foreground_rays(.2)] = np.nan
        stamps = send(pixels=background)
        assert all(m.width == 0 for m in selected(points, stamps))
        assert np.array_equal(bridge.imgmsg_to_cv2(selected(labels, stamps)[-1]), label)
        check_caption(stamps, 'red1: -- mm')
        # Surface continuity is geometric, including a sloped background.
        sloped = (1 / (1 - .3 * (xx - 160) / 250)).astype(np.float32)
        sloped[foreground_rays(.2)] = np.nan
        stamps = send(pixels=sloped)
        assert all(m.width == 0 for m in selected(points, stamps))
        # A measured foreground remains valid, including a 1cm-high object.
        for rgb_z, caption in ((.2, 'red1: 200 mm'), (1.04, 'red1: 1040 mm')):
            measured = np.ones_like(depth)
            measured[foreground_rays(rgb_z)] = rgb_z - .05
            stamps = send(pixels=measured)
            cloud = next(m for m in reversed(selected(points, stamps)) if m.width)
            assert xyz(cloud)[2] == pytest.approx(rgb_z, abs=.002)
            check_caption(stamps, caption)
        # A small near-depth cluster must not displace the supported object surface.
        measured[112:115, 122:125] = .15
        stamps = send(pixels=measured)
        cloud = next(m for m in reversed(selected(points, stamps)) if m.width)
        assert xyz(cloud)[2] == pytest.approx(1.04, abs=.002)
        check_caption(stamps, 'red1: 1040 mm')
        # A short gap must not manufacture observations. Sustained loss clears
        # geometry after the configured grace period; new input then recovers.
        points.clear()
        objects.clear()
        send(pixels=None)
        assert not points and not objects
        spin(settings['depth_localizer']['ros__parameters']['input_timeout'])
        assert points and all(m.width == 0 for m in points)
        assert objects and all(not m.objects for m in objects)

        # Invalid calibration must not replace the last valid calibration.
        bad_di = copy.deepcopy(di)
        bad_di.d = [0.] * 4
        stamps = send(depth_info=bad_di)
        assert any(m.width == 1 for m in selected(points, stamps))
        # An off-axis fisheye marker: project independent 3D marker corners,
        # then verify the recovered pose rather than only accepting its model name.
        fish_ci = copy.deepcopy(ci)
        fish_ci.distortion_model, fish_ci.d = 'equidistant', [.1, -.01, .001, 0.]
        object_corners = np.array([[-.05, .05, 0], [.05, .05, 0],
                                   [.05, -.05, 0], [-.05, -.05, 0]], np.float64)
        image_corners, _ = cv2.fisheye.projectPoints(
            object_corners.reshape(1, 4, 3), np.array([np.pi, 0., 0.]),
            np.array([.12, .04, .5]), np.array(ci.k).reshape(3, 3), np.array(fish_ci.d))
        source = np.array([[0, 0], [99, 0], [99, 99], [0, 99]], np.float32)
        homography = cv2.getPerspectiveTransform(
            source, image_corners.reshape(4, 2).astype(np.float32))
        fish_marker = cv2.warpPerspective(marker, homography, (640, 480),
                                          flags=cv2.INTER_NEAREST, borderValue=255)
        stamps = send(fish_ci, di, raw_pixels=cv2.cvtColor(fish_marker, cv2.COLOR_GRAY2BGR))
        poses = [t for t in selected(transforms, stamps) if t.child_frame_id == 'aruco_marker_7']
        assert poses
        pose = poses[-1].transform
        assert [pose.translation.x, pose.translation.y, pose.translation.z] == pytest.approx(
            [.12, .04, .5], abs=.015)
        assert abs(pose.rotation.x) == pytest.approx(1, abs=.03)

        # Nonzero fisheye depth and RGB use their own calibrations and different grids.
        fish_di = copy.deepcopy(di)
        fish_di.distortion_model, fish_di.d = 'equidistant', [.1, -.01, .001, 0.]
        stamps = send(fish_ci, fish_di)
        assert any(m.width for m in selected(points, stamps))
        bad_ci = copy.deepcopy(ci)
        bad_ci.width = 320
        stamps = send(bad_ci, di)
        assert selected(labels, stamps)
        assert all(m.width == 0 for m in selected(points, stamps))
        stamps = send(ci, di)
        assert any(m.width for m in selected(points, stamps))
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        del subscriptions


def test_wrapper_sigint_without_opening_device(tmp_path):
    """Invalid frame parameters keep hardware unopened; SIGINT must exit cleanly."""
    executable = str(Path(get_package_prefix('vidu_ros2_wrapper')) /
                     'lib' / 'vidu_ros2_wrapper' / 'vidu_ros2_wrapper_node')
    for attempt in range(3):
        path = tmp_path / f'shutdown_{attempt}.log'
        with path.open('w') as log:
            process = subprocess.Popen([
                executable, '--ros-args', '-p', 'color_frame_id:=same_frame',
                '-p', 'depth_frame_id:=same_frame'], stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 8
                while 'Invalid optical frame parameters' not in path.read_text():
                    assert process.poll() is None, path.read_text()
                    assert time.monotonic() < deadline, path.read_text()
                    time.sleep(.02)
                process.send_signal(signal.SIGINT)
                assert process.wait(timeout=5) == 0, path.read_text()
                assert 'Camera connected' not in path.read_text()
                assert 'terminate called' not in path.read_text()
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)


@pytest.mark.parametrize('enable_detection,scale', [(True, 1), (True, 2), (False, 1)])
def test_rgb_detection_and_preview(tmp_path, enable_detection, scale):
    """RGB detection still works; preview has no depth dependency or detection output."""
    executable = str(Path(get_package_prefix('my_robot_perception')) /
                     'lib' / 'my_robot_perception' / 'color_detector')
    rclpy.init()
    node = rclpy.create_node('rgb_only_color_test')
    publisher = node.create_publisher(Image, '/rgb_only/image', 10)
    received = []
    subscription = node.create_subscription(Image, '/rgb_only/labels', received.append, 10)
    bridge = CvBridge()
    pixels = np.zeros((64 * scale, 64 * scale, 3), np.uint8)
    blue = (255, 0, 0) if scale == 1 else tuple(int(v) for v in cv2.cvtColor(
        np.uint8([[[107, 84, 142]]]), cv2.COLOR_HSV2BGR)[0, 0])
    pixels[20 * scale:27 * scale, 20 * scale:27 * scale] = blue
    with (tmp_path / 'rgb_only.log').open('w') as log:
        process = subprocess.Popen([
            executable, '--ros-args',
            '-p', 'image_topic:=/rgb_only/image', '-p', 'labels_topic:=/rgb_only/labels',
            '-p', f'enable_detection:={str(enable_detection).lower()}',
            '-p', 'area_reference_size:=[64,64]', '-p', 'max_area:=100',
            '-p', 'show_window:=false'], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                assert process.poll() is None
                rclpy.spin_once(node, timeout_sec=.05)
            assert publisher.get_subscription_count() == 1
            deadline = time.monotonic() + (3 if enable_detection else .5)
            while not received and time.monotonic() < deadline:
                assert process.poll() is None
                msg = bridge.cv2_to_imgmsg(pixels, encoding='bgr8')
                msg.header.frame_id = 'rgb_optical'
                publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=.05)
            if enable_detection:
                assert received
                assert bridge.imgmsg_to_cv2(received[-1])[23 * scale, 23 * scale] == 3
            else:
                assert not received
                assert node.count_publishers('/rgb_only/labels') == 0
                assert node.count_publishers('/perception/color_debug') == 0
                assert node.count_subscribers('/unused/depth') == 0
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
            del subscription


@pytest.mark.parametrize('executable,name,value', [
    ('color_detector', 'min_area', 1000),
    ('depth_localizer', 'min_depth', .005),
    ('aruco_detector', 'marker_size', .1),
])
def test_startup_parameters_are_readonly(tmp_path, executable, name, value):
    """Reject ineffective runtime changes instead of reporting false success."""
    binary = str(Path(get_package_prefix('my_robot_perception')) /
                 'lib' / 'my_robot_perception' / executable)
    rclpy.init()
    node = rclpy.create_node('readonly_parameter_test')
    with (tmp_path / 'readonly.log').open('w') as log:
        process = subprocess.Popen([binary, '--ros-args', '-p', 'show_window:=false'],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            setter = node.create_client(SetParameters, f'/{executable}/set_parameters')
            getter = node.create_client(GetParameters, f'/{executable}/get_parameters')
            assert setter.wait_for_service(timeout_sec=5)
            assert getter.wait_for_service(timeout_sec=5)
            before = getter.call_async(GetParameters.Request(names=[name]))
            rclpy.spin_until_future_complete(node, before, timeout_sec=3)
            request = SetParameters.Request(
                parameters=[Parameter(name, value=value).to_parameter_msg()])
            future = setter.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and not future.result().results[0].successful
            after = getter.call_async(GetParameters.Request(names=[name]))
            rclpy.spin_until_future_complete(node, after, timeout_sec=3)
            assert before.result().values == after.result().values
            assert process.poll() is None
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


def test_annotation_validation_and_rgb_watchdog(tmp_path):
    """Bad feedback cannot crash RGB detection; valid endian/stride variants still work."""
    binary = str(Path(get_package_prefix('my_robot_perception')) /
                 'lib' / 'my_robot_perception' / 'color_detector')
    rclpy.init()
    node = rclpy.create_node('annotation_validation_test')
    labels, debug = [], []
    subscriptions = [
        node.create_subscription(Image, '/guard/labels', labels.append, 10),
        node.create_subscription(Image, '/guard/debug', debug.append, 10)]
    rgb_pub = node.create_publisher(Image, '/guard/rgb', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/guard/points', 10)
    bridge = CvBridge()
    pixels = np.zeros((128, 128, 3), np.uint8)
    pixels[35:50, 20:35] = (255, 0, 0)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.005)

    def image_with_caption(caption, stale=False):
        image = pixels.copy()
        cv2.rectangle(image, (20, 35), (34, 49), (255, 0, 0), 2)
        cv2.circle(image, (27, 42), 3, (255, 0, 0), -1)
        cv2.putText(image, caption, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 0), 1)
        if stale:
            cv2.putText(image, 'RGB INPUT STALE', (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        .7, (0, 0, 255), 2)
        return image

    with (tmp_path / 'annotation.log').open('w') as log:
        process = subprocess.Popen([
            binary, '--ros-args', '-p', 'image_topic:=/guard/rgb',
            '-p', 'labels_topic:=/guard/labels', '-p', 'debug_topic:=/guard/debug',
            '-p', 'depth_points_topic:=/guard/points', '-p', 'input_timeout:=0.4',
            '-p', 'show_window:=false'], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and (
                    rgb_pub.get_subscription_count() == 0 or
                    cloud_pub.get_subscription_count() == 0):
                spin(.02)
            assert rgb_pub.get_subscription_count() == 1
            for case in ('truncated', 'bad_type', 'bad_offset', 'bad_stride',
                         'missing_field', 'zero_step', 'little', 'big_padded_rows'):
                labels.clear()
                msg = bridge.cv2_to_imgmsg(pixels, 'bgr8')
                msg.header.frame_id = 'guard_optical_frame'
                msg.header.stamp = node.get_clock().now().to_msg()
                rgb_pub.publish(msg)
                deadline = time.monotonic() + 1
                while not labels and time.monotonic() < deadline:
                    spin(.005)
                assert labels and process.poll() is None
                cloud = PointCloud2()
                cloud.header = msg.header
                cloud.width = cloud.height = 1
                cloud.point_step = cloud.row_step = 13
                cloud.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in
                                [('u', 0, 6), ('v', 4, 6), ('depth', 8, 7), ('label', 12, 2)]]
                cloud.data = struct.pack('<IIfB', 22, 37, .15, 3)
                valid = case in ('little', 'big_padded_rows')
                if case == 'truncated':
                    cloud.data = []
                elif case == 'bad_type':
                    cloud.fields[0].datatype = PointField.FLOAT32
                elif case == 'bad_offset':
                    cloud.fields[0].offset = 13
                elif case == 'bad_stride':
                    cloud.row_step = 12
                elif case == 'missing_field':
                    cloud.fields = cloud.fields[:-1]
                elif case == 'zero_step':
                    cloud.point_step = 0
                elif case == 'big_padded_rows':
                    cloud.is_bigendian, cloud.height, cloud.row_step = True, 2, 20
                    row = struct.pack('>IIfB', 22, 37, .15, 3) + bytes(7)
                    cloud.data = row * 2
                cloud_pub.publish(cloud)
                deadline = time.monotonic() + 1
                selected = []
                while not selected and time.monotonic() < deadline:
                    spin(.01)
                    selected = [m for m in debug if m.header.stamp == msg.header.stamp]
                assert selected and process.poll() is None, (
                    tmp_path / 'annotation.log').read_text()
                expected = image_with_caption('blue1: 150 mm' if valid else 'blue1: -- mm')
                assert np.array_equal(bridge.imgmsg_to_cv2(selected[-1], 'bgr8'), expected)
            # No new RGB: old numeric depth is cleared once, with an explicit stale banner.
            spin(.6)
            assert np.array_equal(bridge.imgmsg_to_cv2(debug[-1], 'bgr8'),
                                  image_with_caption('blue1: -- mm', stale=True))
            assert process.poll() is None
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
            del subscriptions


@pytest.mark.parametrize('tilt_x,tilt_y', [(20, 0), (0, 30), (20, 25)])
def test_oblique_cube_center_with_depth_holes(tmp_path, tilt_x, tilt_y):
    """A ray-rendered cube has a known centre even when its sides and holes are visible."""
    width, height, focal = 320, 240, 350.
    centre = np.array([.04, -.03, .025])
    size = .05
    rx, _ = cv2.Rodrigues(np.array([np.deg2rad(tilt_x), 0., 0.]))
    ry, _ = cv2.Rodrigues(np.array([0., np.deg2rad(tilt_y), 0.]))
    rotation = ry @ rx @ np.diag([1., -1., -1.])
    origin = centre - rotation @ np.array([0., 0., .45])
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    rays = np.stack([(u - width/2) / focal, (v - height/2) / focal,
                     np.ones_like(u)], axis=-1) @ rotation.T
    safe = np.where(np.abs(rays) < 1e-12, 1e-12, rays)
    a, b = (centre-size/2-origin) / safe, (centre+size/2-origin) / safe
    near = np.minimum(a, b).max(axis=-1)
    far = np.maximum(a, b).min(axis=-1)
    visible = (near > 0) & (far >= near)
    labels = visible.astype(np.uint8)
    depth = np.where(visible, near, -origin[2] / rays[..., 2]).astype(np.float32)
    ys, xs = np.where(visible)
    assert xs.min() > 0 and xs.max() < width-1 and ys.min() > 0 and ys.max() < height-1

    rclpy.init()
    node = rclpy.create_node('oblique_cube_probe')
    bridge = CvBridge()
    objects = []
    from my_robot_interface.msg import DetectedObjectArray
    subscription = node.create_subscription(DetectedObjectArray, '/perception/objects', objects.append, 10)
    label_pub = node.create_publisher(Image, '/perception/color_labels', 10)
    depth_pub = node.create_publisher(Image, '/camera/depth', 10)
    depth_info_pub = node.create_publisher(CameraInfo, '/camera/info/depth', 10)
    color_info_pub = node.create_publisher(CameraInfo, '/camera/info/color', 10)
    extrinsics_pub = node.create_publisher(Extrinsics, '/camera/extrinsics/color_to_depth', 10)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'camera_color_optical_frame'
    transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, origin)
    rv, _ = cv2.Rodrigues(rotation)
    angle = np.linalg.norm(rv)
    q = rv.ravel() / angle * np.sin(angle/2)
    transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z = map(float, q)
    transform.transform.rotation.w = float(np.cos(angle/2))
    broadcaster.sendTransform(transform)

    def camera_info(frame):
        info = CameraInfo()
        info.header.frame_id = frame
        info.width, info.height = width, height
        info.distortion_model, info.d = 'plumb_bob', [0.] * 5
        info.k = [focal, 0., width/2, 0., focal, height/2, 0., 0., 1.]
        info.p = [focal, 0., width/2, 0., 0., focal, height/2, 0., 0., 0., 1., 0.]
        return info

    di, ci = camera_info('camera_depth_optical_frame'), camera_info('camera_color_optical_frame')
    extrinsics = Extrinsics()
    extrinsics.rotation, extrinsics.translation = np.eye(3).ravel().tolist(), [0.] * 3
    extrinsics.header.frame_id = ci.header.frame_id
    config = yaml.safe_load((ROOT / 'config/perception.yaml').read_text())
    config['depth_localizer']['ros__parameters']['object_dimensions'] = [size] * 3
    config_path = tmp_path / 'perception.yaml'
    config_path.write_text(yaml.safe_dump(config))
    binary = Path(get_package_prefix('my_robot_perception')) / 'lib/my_robot_perception/depth_localizer'
    log_path = tmp_path / 'oblique.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config_path),
                                    '--log-level', 'depth_localizer:=debug'],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            for holes in (False, True):
                pixels = depth.copy()
                if holes:
                    pixels[visible & (u > np.median(xs))] = np.nan
                objects.clear()
                after = node.get_clock().now().nanoseconds
                deadline = time.monotonic() + 8
                found = []
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    stamp = node.get_clock().now().to_msg()
                    for pub, info in ((depth_info_pub, di), (color_info_pub, ci)):
                        info.header.stamp = stamp
                        pub.publish(info)
                    extrinsics.header.stamp = stamp
                    extrinsics_pub.publish(extrinsics)
                    label = bridge.cv2_to_imgmsg(labels, 'mono8')
                    label.header.stamp, label.header.frame_id = stamp, ci.header.frame_id
                    image = bridge.cv2_to_imgmsg(pixels, '32FC1')
                    image.header.stamp, image.header.frame_id = stamp, di.header.frame_id
                    label_pub.publish(label)
                    depth_pub.publish(image)
                    rclpy.spin_once(node, timeout_sec=.05)
                    found = [o for message in objects
                             if message.header.stamp.sec*10**9 + message.header.stamp.nanosec >= after
                             for o in message.objects if o.color == 1]
                    if found:
                        break
                assert found, f'holes={holes}\n' + log_path.read_text()
                actual = found[-1].pose.position
                assert [actual.x, actual.y, actual.z] == pytest.approx(centre, abs=.002)
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
