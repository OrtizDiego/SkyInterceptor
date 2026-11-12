#!/usr/bin/env python3
"""
Generate the auto-labelled UAS dataset from the simulator (plan 3.2).

The intruder of the INTERCEPT scenario (``target_drone``, a Crazyflie) is a
known rigid body at a known pose in the world, so its box in the image can be
computed from the ground truth of the simulator instead of being drawn by
hand. The generator:

1. flies the interceptor to a set of viewpoints, so the background, the
   distance and the bearing of the intruder change;
2. places the intruder at a set of poses, and lets it fly the five
   behaviours of plan 3.1;
3. takes the stereo left image and computes the box of the intruder by
   projecting the eight corners of its bounding box into the camera with the
   camera model the 3D localizer uses;
4. writes the images and the labels in the YOLO format.

A frame is labelled as *background* (an empty label file) when the intruder
is not visible: behind the camera, outside of the frame, or hidden behind a
tree. The decision is taken on the ground truth, never on the image, so an
intruder that cannot be seen never turns into a negative box.

Usage (inside the container, with the workspace sourced):

    python3 scripts/uas_dataset/generate_dataset.py --out /tmp/uas_dataset \\
        --seed 1

The script starts the simulator itself; pass ``--no-sim`` to use one that is
already running.
"""

import argparse
import csv
import math
import os
import random
import signal
import socket
import subprocess
import sys
import time

import numpy as np
import rclpy
import yaml
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'dataset_config.yaml')

CAMERA_FRAME = 'stereo_left_camera_optical_frame'
TARGET_FRAME = 'target_base_link'
WORLD_FRAME = 'map'
TARGET_ENTITY = 'target_drone'


def quat_to_matrix(quaternion):
    """The rotation matrix of a geometry_msgs/Quaternion."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(quaternion):
    """The yaw of a geometry_msgs/Quaternion (ZYX convention)."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def decode_image(message):
    """Decode a sensor_msgs/Image into an (H, W, 3) uint8 RGB array."""
    encoding = message.encoding.lower()
    if encoding == 'rgb8':
        channels, swap = 3, False
    elif encoding == 'bgr8':
        channels, swap = 3, True
    else:
        raise ValueError('unsupported image encoding: ' + message.encoding)
    row_bytes = message.width * channels
    data = np.frombuffer(message.data, dtype=np.uint8)
    rows = data[:message.height * message.step].reshape(message.height, message.step)
    pixels = rows[:, :row_bytes].reshape(message.height, message.width, channels)
    if swap:
        pixels = pixels[:, :, ::-1]
    return np.ascontiguousarray(pixels)


class UasDatasetGenerator(Node):
    """Drives the simulator and writes the labelled image set."""

    def __init__(self, config, args):
        super().__init__(
            'uas_dataset_generator',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.config = config
        self.args = args
        self.rng = random.Random(args.seed)

        self.image = None
        self.image_count = 0
        self.image_time = 0.0
        self.camera_info = None
        self.odom = None
        self.simulator = None

        # A depth of one: only the newest frame is of any use, and the camera
        # publishes at 60 Hz.
        self.create_subscription(
            Image, '/stereo/left/image_raw', self.on_image, 1)
        self.create_subscription(
            CameraInfo, '/stereo/left/camera_info', self.on_camera_info, 10)
        self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)

        self.arm_publisher = self.create_publisher(Bool, '/drone/arm', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.entity_client = self.create_client(SetEntityState, '/set_entity_state')

        self.control_timer = self.create_timer(0.02, self.on_control_timer)
        self.position_desired = np.zeros(3)
        self.yaw_desired = 0.0
        self.armed = False

        # The pose of the intruder while the generator places it itself. The
        # behaviour node publishes the same transform, so the generator keeps
        # quiet about it while the intruder flies a behaviour.
        self.own_target = True
        self.target_position = np.zeros(3)
        self.target_yaw = 0.0

        self.records = []
        self.images_written = 0
        self.crops_written = 0
        self.prefix = args.prefix
        self.splits = self.assign_splits()

    # --- callbacks ---------------------------------------------------------

    def on_image(self, message):
        """Keep the newest image."""
        self.image = message
        self.image_count += 1
        self.image_time = time.time()

    def on_camera_info(self, message):
        """Keep the newest camera info."""
        self.camera_info = message

    def on_odom(self, message):
        """Keep the newest odometry."""
        self.odom = message

    # --- helpers ------------------------------------------------------------

    def now(self):
        """The current time of the node clock (the simulation time)."""
        return self.get_clock().now()

    def sim_deadline(self, seconds):
        """A deadline on the clock of the simulation."""
        return self.now() + rclpy.duration.Duration(seconds=seconds)

    def log(self, text):
        """Log one line of the progress of the generator."""
        self.get_logger().info(text)
        sys.stdout.flush()

    def intrinsics(self):
        """The camera intrinsics, from the camera info when there is one."""
        if self.camera_info is not None:
            k = self.camera_info.k
            return (k[0], k[4], k[2], k[5],
                    self.camera_info.width, self.camera_info.height)
        camera = self.config['camera']
        return (camera['fx'], camera['fy'], camera['cx'], camera['cy'],
                camera['width'], camera['height'])

    def lookup(self, source_frame, stamp):
        """
        The transform of a frame in the world frame.

        The pose is looked up at the stamp of the image. The buffer may not
        have a sample that far in the future yet, in which case the newest
        transform is used.
        """
        try:
            return self.tf_buffer.lookup_transform(
                WORLD_FRAME, source_frame, stamp)
        except Exception:
            return self.tf_buffer.lookup_transform(
                WORLD_FRAME, source_frame, rclpy.time.Time())

    def camera_pose(self, stamp=None):
        """The pose of the camera in the world frame."""
        if stamp is None:
            stamp = self.now()
        transform = self.lookup(CAMERA_FRAME, stamp)
        rotation = quat_to_matrix(transform.transform.rotation)
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z])
        return rotation, translation

    def target_corners(self, stamp):
        """The eight corners of the intruder in the world frame."""
        target = self.config['target']
        local = np.array([
            [sx * target['half_x'], sy * target['half_y'], z]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for z in (target['z_min'], target['z_max'])])
        transform = self.lookup(TARGET_FRAME, stamp)
        rotation = quat_to_matrix(transform.transform.rotation)
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z])
        return local @ rotation.T + translation, translation

    def occluded(self, camera_position, target_position):
        """True when a tree stands between the camera and the intruder."""
        dx = target_position[0] - camera_position[0]
        dy = target_position[1] - camera_position[1]
        dz = target_position[2] - camera_position[2]
        length_squared = dx * dx + dy * dy
        if length_squared < 1e-9:
            return False
        for tree in self.config['trees']:
            x, y, radius, height = tree
            # The parameter of the point of the line that is closest to the
            # axis of the cylinder.
            t = ((x - camera_position[0]) * dx + (y - camera_position[1]) * dy)
            t = min(max(t / length_squared, 0.0), 1.0)
            closest_x = camera_position[0] + t * dx
            closest_y = camera_position[1] + t * dy
            closest_z = camera_position[2] + t * dz
            if (closest_x - x) ** 2 + (closest_y - y) ** 2 > radius * radius:
                continue
            if 0.0 <= closest_z <= height:
                return True
        return False

    def label_of(self, message):
        """
        The label of one image: (class, xc, yc, w, h) or None for the
        background, together with the reason why a frame is a background.
        """
        offset = self.config['sampling']['tf_time_offset']
        stamp = rclpy.time.Time.from_msg(
            message.header.stamp, clock_type=self.get_clock().clock_type)
        stamp = stamp + rclpy.duration.Duration(seconds=offset)
        try:
            corners, target_position = self.target_corners(stamp)
            rotation, camera_position = self.camera_pose(stamp)
        except Exception:
            return None, 'no_transform'

        # The optical frame looks along +z, +x is to the right and +y is down.
        in_camera = (corners - camera_position) @ rotation
        if np.any(in_camera[:, 2] < self.config['camera']['near']):
            return None, 'behind_camera'

        fx, fy, cx, cy, width, height = self.intrinsics()
        u = fx * in_camera[:, 0] / in_camera[:, 2] + cx
        v = fy * in_camera[:, 1] / in_camera[:, 2] + cy

        x_min = max(float(u.min()), 0.0)
        x_max = min(float(u.max()), float(width))
        y_min = max(float(v.min()), 0.0)
        y_max = min(float(v.max()), float(height))
        if x_max <= x_min or y_max <= y_min:
            return None, 'outside_frame'

        # A frame in which only a sliver of the intruder is visible is
        # labelled as the background: the box would be mostly empty.
        full_area = (u.max() - u.min()) * (v.max() - v.min())
        visible_area = (x_max - x_min) * (y_max - y_min)
        if full_area <= 0.0 or visible_area < 0.5 * full_area:
            return None, 'partly_outside_frame'

        if self.occluded(camera_position, target_position):
            return None, 'occluded'

        box_width = x_max - x_min
        box_height = y_max - y_min
        label = (
            0,
            (x_min + box_width / 2.0) / width,
            (y_min + box_height / 2.0) / height,
            box_width / width,
            box_height / height,
        )
        return label, 'visible'

    # --- the flight of the interceptor --------------------------------------

    def on_control_timer(self):
        """The outer position loop: the position error becomes a velocity."""
        if self.own_target:
            self.publish_target_transform()
        if self.odom is None:
            return
        twist = Twist()
        if self.armed:
            position = self.odom.pose.pose.position
            current = np.array([position.x, position.y, position.z])
            error = self.position_desired - current
            speed = self.config['flight']
            horizontal = error[:2] * speed['position_gain']
            norm = float(np.linalg.norm(horizontal))
            if norm > speed['max_speed']:
                horizontal = horizontal * (speed['max_speed'] / norm)
            vertical = float(np.clip(
                error[2] * speed['position_gain'],
                -speed['max_climb'], speed['max_climb']))
            twist.linear.x = float(horizontal[0])
            twist.linear.y = float(horizontal[1])
            twist.linear.z = vertical
            error_yaw = wrap_angle(
                self.yaw_desired - yaw_of(self.odom.pose.pose.orientation))
            twist.angular.z = float(np.clip(
                error_yaw * speed['yaw_gain'],
                -speed['max_yaw_rate'], speed['max_yaw_rate']))
        self.velocity_publisher.publish(twist)

    def arm(self):
        """Start the motors of the interceptor."""
        for _ in range(5):
            self.arm_publisher.publish(Bool(data=True))
            time.sleep(0.1)
        self.armed = True
        self.log('The interceptor is armed')

    def position_of_interceptor(self):
        """The position of the interceptor in the world frame."""
        position = self.odom.pose.pose.position
        return np.array([position.x, position.y, position.z])

    def fly_to(self, position, yaw, settle=None):
        """Command a new viewpoint and wait for the interceptor to get there."""
        self.position_desired = np.array(position, dtype=float)
        self.yaw_desired = yaw
        if settle is None:
            settle = self.config['flight']['settle_time']
        # The clock of the simulation: the wall clock is not a good measure
        # here, because the simulator does not always run in real time.
        deadline = self.sim_deadline(settle)
        wall_deadline = time.time() + 3.0 * settle + 30.0
        while self.now() < deadline and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is None:
                continue
            error = np.linalg.norm(
                self.position_of_interceptor() - self.position_desired)
            if error < 0.5:
                break

    # --- the intruder --------------------------------------------------------

    def set_target_pose(self, position, yaw):
        """Move the intruder to a pose in the world frame."""
        if not self.entity_client.service_is_ready():
            self.log('Waiting for /set_entity_state')
            return False
        self.target_position = np.array(position, dtype=float)
        self.target_yaw = yaw
        request = SetEntityState.Request()
        request.state.name = TARGET_ENTITY
        request.state.reference_frame = 'world'
        request.state.pose.position.x = float(position[0])
        request.state.pose.position.y = float(position[1])
        request.state.pose.position.z = float(position[2])
        request.state.pose.orientation.x = 0.0
        request.state.pose.orientation.y = 0.0
        request.state.pose.orientation.z = math.sin(yaw / 2.0)
        request.state.pose.orientation.w = math.cos(yaw / 2.0)
        future = self.entity_client.call_async(request)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            return False
        return bool(future.result().success)

    def publish_target_transform(self):
        """Publish the map -> target_base_link transform of the intruder."""
        transform = TransformStamped()
        transform.header.stamp = self.now().to_msg()
        transform.header.frame_id = WORLD_FRAME
        transform.child_frame_id = TARGET_FRAME
        transform.transform.translation.x = float(self.target_position[0])
        transform.transform.translation.y = float(self.target_position[1])
        transform.transform.translation.z = float(self.target_position[2])
        transform.transform.rotation.z = math.sin(self.target_yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.target_yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def sample_pose(self, camera_rotation, camera_position):
        """
        A pose of the intruder in front of the camera.

        The distance is sampled logarithmically, because the engagement
        happens at a few metres and the far end of the range only needs a few
        samples. A share of the samples puts the intruder outside of the
        field of view: those frames are the background ones.
        """
        sampling = self.config['sampling']
        negative = self.rng.random() < sampling['negative_fraction']
        distance = math.exp(self.rng.uniform(
            math.log(sampling['distance'][0]), math.log(sampling['distance'][1])))
        if negative:
            bearing = math.radians(self.rng.choice([
                self.rng.uniform(32.0, 80.0),
                self.rng.uniform(-80.0, -32.0),
                self.rng.uniform(140.0, 180.0),
            ]))
        else:
            bearing = math.radians(self.rng.uniform(*sampling['bearing']))
        elevation = math.radians(self.rng.uniform(*sampling['elevation']))
        yaw = self.rng.uniform(-math.pi, math.pi)

        forward = camera_rotation @ np.array([0.0, 0.0, 1.0])
        right = camera_rotation @ np.array([1.0, 0.0, 0.0])
        up = camera_rotation @ np.array([0.0, -1.0, 0.0])
        direction = (math.cos(elevation) * math.cos(bearing) * forward +
                     math.cos(elevation) * math.sin(bearing) * right +
                     math.sin(elevation) * up)
        position = camera_position + direction * distance
        return (position, yaw, distance,
                math.degrees(bearing), math.degrees(elevation))

    def inside_tree(self, position):
        """True when the pose of the intruder is inside a canopy."""
        for tree in self.config['trees']:
            x, y, radius, height = tree
            if (position[0] - x) ** 2 + (position[1] - y) ** 2 < radius ** 2:
                if position[2] < height:
                    return True
        return False

    # --- the capture ---------------------------------------------------------

    def wait_for_image(self, after=None, timeout=5.0):
        """
        Wait for an image that is newer than the given one.

        The stamp is the one of the simulation time of the frame, so the
        images come in the order they were rendered.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.image is None:
                continue
            if after is None:
                return self.image
            stamp = rclpy.time.Time.from_msg(
                self.image.header.stamp, clock_type=self.get_clock().clock_type)
            if stamp > after:
                return self.image
        return None

    def write_sample(self, message, scene, mode, meta, name):
        """Write one image and its label, and record it in the manifest."""
        from PIL import Image as PilImage
        scene = self.prefix + scene
        label, reason = self.label_of(message)
        array = decode_image(message)
        filename = '{}{:05d}_{}.png'.format(
            self.prefix, self.images_written, name)
        split = self.splits[scene]
        image_dir = os.path.join(self.args.out, 'images', split)
        label_dir = os.path.join(self.args.out, 'labels', split)
        os.makedirs(image_dir, exist_ok=True)
        os.makedirs(label_dir, exist_ok=True)
        PilImage.fromarray(array).save(os.path.join(image_dir, filename))
        self.write_debug_crop(array, label, filename)
        lines = []
        if label is not None:
            lines.append('{} {:.6f} {:.6f} {:.6f} {:.6f}'.format(*label))
        with open(os.path.join(label_dir, filename.replace('.png', '.txt')),
                  'w', encoding='utf-8') as handle:
            handle.write('\n'.join(lines) + ('\n' if lines else ''))
        self.records.append({
            'image': filename,
            'split': split,
            'scene': scene,
            'mode': mode,
            'reason': reason,
            'distance_m': '{:.2f}'.format(meta.get('distance', 0.0)),
            'bearing_deg': '{:.1f}'.format(meta.get('bearing', 0.0)),
            'elevation_deg': '{:.1f}'.format(meta.get('elevation', 0.0)),
            'box_w': '{:.1f}'.format(0.0 if label is None else label[3] * 640),
            'box_h': '{:.1f}'.format(0.0 if label is None else label[4] * 480),
        })
        self.images_written += 1

    def write_debug_crop(self, array, label, filename):
        """
        Save a crop around the box of the intruder.

        The crop is what the label was computed on, so it is the way to check
        by eye that the box is on the intruder and not somewhere else.
        """
        if label is None or self.crops_written >= self.args.debug_crops:
            return
        height, width = array.shape[:2]
        box_width = max(label[3] * width, 4.0)
        box_height = max(label[4] * height, 4.0)
        centre_x = label[1] * width
        centre_y = label[2] * height
        half = max(box_width, box_height, 24.0) * 1.5
        x_min = max(int(centre_x - half), 0)
        x_max = min(int(centre_x + half), width)
        y_min = max(int(centre_y - half), 0)
        y_max = min(int(centre_y + half), height)
        if x_max <= x_min or y_max <= y_min:
            return
        crop = array[y_min:y_max, x_min:x_max]
        directory = os.path.join(self.args.out, 'debug')
        os.makedirs(directory, exist_ok=True)
        from PIL import Image as PilImage
        PilImage.fromarray(crop).resize(
            (crop.shape[1] * 4, crop.shape[0] * 4),
            PilImage.NEAREST).save(
                os.path.join(directory, filename))
        self.crops_written += 1

    def assign_splits(self):
        """
        Assign the scenes to the train and the validation set.

        The split is done by scene and not by image: the frames of a scene are
        nearly the same picture, and they must not end up on both sides of the
        split.
        """
        scenes = ['sweep_{:02d}'.format(index)
                  for index in range(self.config['sampling']['blocks'])]
        scenes += ['behaviour_{:02d}'.format(index)
                   for index in range(len(self.config['behaviours']))]
        scenes = [self.prefix + scene for scene in scenes]
        shuffled = list(scenes)
        self.rng.shuffle(shuffled)
        fraction = self.config['split']['train_fraction']
        count = max(1, int(round(len(scenes) * fraction)))
        splits = {}
        for index, scene in enumerate(shuffled):
            splits[scene] = 'train' if index < count else 'val'
        return splits

    # --- the simulator --------------------------------------------------------

    def simulator_alive(self):
        """True while the simulator is running and the camera is alive."""
        if self.simulator is None or self.simulator.poll() is not None:
            return False
        return time.time() - self.image_time < 5.0

    def ensure_simulator(self):
        """
        Make sure that the simulator is running.

        The simulator of a shared machine can die (the graphics driver is
        shared with the other simulators of the container), so the generator
        restarts it and takes off again when that happens. The frames that
        were written before the crash stay in the dataset.
        """
        if self.simulator_alive():
            return True
        self.log('The simulator died, it is started again')
        stop_simulator(self.simulator)
        self.simulator = start_simulator()
        self.image = None
        self.odom = None
        self.image_time = 0.0
        if not self.wait_for_simulator(timeout=120.0):
            return False
        self.take_off()
        return True

    # --- the phases -----------------------------------------------------------

    def wait_for_simulator(self, timeout=180.0):
        """Wait for the camera, the odometry and the transform of the camera."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image is None or self.odom is None:
                continue
            try:
                self.camera_pose()
            except Exception:
                continue
            self.log('The simulator is up: {} images so far'.format(
                self.image_count))
            return True
        self.log('The simulator did not come up in time')
        return False

    def take_off(self):
        """Arm the interceptor and climb to the working altitude."""
        flight = self.config['flight']
        altitude = self.rng.uniform(*flight['altitude'])
        self.arm()
        # The climb is straight up: the canopies of the trees are in the way
        # of a climb that drifts sideways, and the interceptor would end up
        # hanging in a tree.
        if self.odom is not None:
            current = self.position_of_interceptor()
            self.position_desired = np.array([current[0], current[1], altitude])
        else:
            self.position_desired = np.array([0.0, 0.0, altitude])
        self.yaw_desired = self.rng.uniform(-math.pi, math.pi)
        deadline = self.sim_deadline(60.0)
        wall_deadline = time.time() + 240.0
        while self.now() < deadline and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is None:
                continue
            if self.position_of_interceptor()[2] > altitude - 0.5:
                break
        height = (self.position_of_interceptor()[2]
                  if self.odom is not None else 0.0)
        if height < altitude - 1.0:
            self.log('The interceptor only reached {:.1f} m'.format(height))
        else:
            self.log('The interceptor is at {:.1f} m'.format(height))
        # Let the vehicle settle before the first sample.
        time.sleep(2.0)

    def sweep(self):
        """
        The pose sweep: the intruder is placed in front of the camera, which
        moves from viewpoint to viewpoint.
        """
        sampling = self.config['sampling']
        flight = self.config['flight']
        area = flight['area']
        for block in range(sampling['blocks']):
            altitude = self.rng.uniform(*flight['altitude'])
            self.position_desired = np.array([
                self.rng.uniform(area[0], area[1]),
                self.rng.uniform(area[2], area[3]),
                altitude])
            self.yaw_desired = self.rng.uniform(-math.pi, math.pi)
            self.fly_to(self.position_desired, self.yaw_desired)
            for _ in range(sampling['samples_per_block']):
                if not self.ensure_simulator():
                    return
                try:
                    rotation, position = self.camera_pose()
                except Exception:
                    continue
                target, yaw, distance, bearing, elevation = self.sample_pose(
                    rotation, position)
                if self.inside_tree(target):
                    continue
                if not self.set_target_pose(target, yaw):
                    continue
                # Wait for the renderer to catch up with the new pose.
                after = self.now() + rclpy.duration.Duration(seconds=0.1)
                message = self.wait_for_image(after=after, timeout=3.0)
                if message is None:
                    continue
                self.write_sample(
                    message, 'sweep_{:02d}'.format(block), 'sweep',
                    {'distance': distance, 'bearing': bearing,
                     'elevation': elevation},
                    'sweep')
            self.log('Viewpoint {}/{}: {} images'.format(
                block + 1, sampling['blocks'], self.images_written))

    def start_behaviour(self, behaviour, position, yaw, speed):
        """Start the behaviour node with the intruder at a pose."""
        command = [
            'ros2', 'run', 'interceptor_drone', 'target_drone_behavior_node',
            '--ros-args',
            '-p', 'behaviour:=' + behaviour,
            '-p', 'start_x:={:.3f}'.format(position[0]),
            '-p', 'start_y:={:.3f}'.format(position[1]),
            '-p', 'start_z:={:.3f}'.format(position[2]),
            '-p', 'start_yaw:={:.3f}'.format(yaw),
            '-p', 'waypoint_speed:={:.2f}'.format(speed),
            '-p', 'weave_speed:={:.2f}'.format(speed),
            '-p', 'final_speed:={:.2f}'.format(speed),
            '-p', 'straight_speed:={:.2f}'.format(speed),
            '-p', 'straight_distance:={:.2f}'.format(60.0),
        ]
        return subprocess.Popen(
            command, env=os.environ.copy(), start_new_session=True)

    def behaviours(self):
        """Let the intruder fly the five behaviours and record the frames."""
        for index, entry in enumerate(self.config['behaviours']):
            if not self.ensure_simulator():
                return
            try:
                rotation, position = self.camera_pose()
            except Exception:
                continue
            target, yaw, distance, bearing, elevation = self.sample_pose(
                rotation, position)
            if self.inside_tree(target):
                continue
            # The behaviour node owns the pose of the intruder, and with it
            # the transform: the generator keeps quiet about it.
            self.own_target = False
            process = self.start_behaviour(
                entry['name'], target, yaw, entry['speed'])
            # Give the behaviour node the time to publish its transform.
            time.sleep(2.0)
            deadline = time.time() + entry['seconds']
            period = 1.0 / self.config['sampling']['capture_rate']
            next_capture = time.time()
            stamp = None
            missed = 0
            while time.time() < deadline:
                message = self.wait_for_image(after=stamp, timeout=1.0)
                if message is None:
                    # No new frame: the camera of the simulator stopped, so
                    # the pose of the intruder is no longer the one that is in
                    # the image.
                    missed += 1
                    if missed > 2:
                        break
                    continue
                missed = 0
                stamp = rclpy.time.Time.from_msg(
                    message.header.stamp,
                    clock_type=self.get_clock().clock_type)
                if time.time() < next_capture:
                    continue
                next_capture = time.time() + period
                self.write_sample(
                    message, 'behaviour_{:02d}'.format(index), entry['name'],
                    {'distance': distance, 'bearing': bearing,
                     'elevation': elevation},
                    entry['name'])
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                process.wait(timeout=10)
            except Exception:
                process.kill()
            self.own_target = True
            self.log('Behaviour {}: {} images'.format(
                entry['name'], self.images_written))

    def load_manifest(self):
        """Read the manifest of a dataset that is being extended."""
        manifest = os.path.join(self.args.out, 'manifest.csv')
        if not os.path.exists(manifest):
            return
        with open(manifest, 'r', newline='', encoding='utf-8') as handle:
            self.records = list(csv.DictReader(handle))
        self.log('The dataset already holds {} images'.format(len(self.records)))

    def write_dataset(self):
        """Write the manifest and the data.yaml of the dataset."""
        manifest = os.path.join(self.args.out, 'manifest.csv')
        with open(manifest, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=[
                'image', 'split', 'scene', 'mode', 'reason', 'distance_m',
                'bearing_deg', 'elevation_deg', 'box_w', 'box_h'])
            writer.writeheader()
            for record in self.records:
                writer.writerow(record)
        data = {
            'path': os.path.abspath(self.args.out),
            'train': 'images/train',
            'val': 'images/val',
            'names': {0: 'uav'},
        }
        with open(os.path.join(self.args.out, 'data.yaml'), 'w',
                  encoding='utf-8') as handle:
            yaml.safe_dump(data, handle, default_flow_style=False)
        positive = sum(1 for record in self.records
                       if record['reason'] == 'visible')
        self.log('{} images, {} with the intruder, {} background'.format(
            len(self.records), positive, len(self.records) - positive))


def free_port():
    """A free TCP port on the loopback interface."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as handle:
        handle.bind(('127.0.0.1', 0))
        return handle.getsockname()[1]


def start_simulator():
    """
    Start the simulator and return the process.

    The Gazebo master binds a fixed port (11345 by default), so a second
    simulator in the same machine cannot start. The generator therefore gives
    its simulator a free port of its own.
    """
    environment = os.environ.copy()
    if 'GAZEBO_MASTER_URI' not in environment:
        environment['GAZEBO_MASTER_URI'] = '127.0.0.1:{}'.format(free_port())
    # The generator flies the interceptor with a world-frame position loop, so
    # the simulator has to take the command in the world frame as well.
    command = [
        'ros2', 'launch', 'interceptor_drone', 'simulation.launch.py',
        'gui:=false', 'command_frame:=world']
    print('The simulator runs on the Gazebo master ' +
          environment['GAZEBO_MASTER_URI'], flush=True)
    return subprocess.Popen(
        command, env=environment, start_new_session=True)


def stop_simulator(process):
    """Stop the simulator and everything it started."""
    if process is None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=30)
    except Exception:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except Exception:
            pass


def main():
    """Generate the dataset."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True,
                        help='the directory the dataset is written to')
    parser.add_argument('--seed', type=int, default=0,
                        help='the seed of the random generator')
    parser.add_argument('--config', default=CONFIG_PATH,
                        help='the configuration of the generator')
    parser.add_argument('--no-sim', action='store_true',
                        help='use a simulator that is already running')
    parser.add_argument('--append', action='store_true',
                        help='add the images to a dataset that is already '
                             'there, instead of replacing it')
    parser.add_argument('--prefix', default='',
                        help='a prefix for the names of the scenes and of the '
                             'files, so several runs can be merged into one '
                             'dataset')
    parser.add_argument('--debug-crops', type=int, default=0,
                        help='save this many crops around the box of the '
                             'intruder, to check the labels by eye')
    args = parser.parse_args()

    with open(args.config, 'r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)

    os.makedirs(args.out, exist_ok=True)
    manifest = os.path.join(args.out, 'manifest.csv')
    if os.path.exists(manifest) and not args.append:
        print('{} already holds a dataset: use --append or another '
              'directory'.format(args.out), file=sys.stderr)
        return 1
    simulator = None
    if not args.no_sim:
        simulator = start_simulator()

    rclpy.init(args=None)
    node = None
    try:
        node = UasDatasetGenerator(config, args)
        node.simulator = simulator
        if args.append:
            node.load_manifest()
        if not node.wait_for_simulator():
            return 1
        node.take_off()
        node.sweep()
        node.behaviours()
        node.write_dataset()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        stop_simulator(simulator)
    return 0


if __name__ == '__main__':
    sys.exit(main())
