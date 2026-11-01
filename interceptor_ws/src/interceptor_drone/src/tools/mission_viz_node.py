#!/usr/bin/env python3
"""
Draw what the pipeline is doing behind the Gazebo scene, for RViz.

The pipeline's own messages (TargetDetection, TargetStateArray, FlightSetpoint,
MissionStatus, GuidanceCommand) are custom types RViz cannot display. This node
reads them and republishes what they mean as markers and paths, one topic per
layer, so rviz/interceptor_config.rviz can show and hide each layer on its own:

    /viz/perception   the raw 3D detections (a fading cloud: the measurements
                      the tracker fuses) and the stereo camera's field of view
    /viz/estimation   the covariance ellipsoid of every track and the predicted
                      path of the selected target
    /viz/guidance     the range line to the target, the planner's raw velocity
                      command (the safety filter's arrow next to it shows what
                      it let through), the capture envelope and the altitude
                      drop lines
    /viz/hud          the mission state as text floating above the interceptor
    /viz/world        the geofence

    /viz/interceptor_path       the interceptor's trail (from /odom)
    /viz/target_truth_path      the intruder's true trail (/target_drone/odom,
                                the simulator's ground truth)
    /viz/target_estimate_path   the tracker's estimate of the selected target

Nothing in the pipeline reads these topics: the node only watches.

    ros2 run interceptor_drone mission_viz_node.py
"""

from collections import deque
import math

from geometry_msgs.msg import Point, PoseStamped, Vector3Stamped
from interceptor_interfaces.msg import (
    FlightSetpoint, GuidanceCommand, MissionMode, MissionStatus,
    TargetDetection, TargetState, TargetStateArray)
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

MODE_NAMES = {
    MissionMode.MODE_HOLD: 'HOLD',
    MissionMode.MODE_FOLLOW: 'FOLLOW',
    MissionMode.MODE_INTERCEPT: 'INTERCEPT',
}

PHASE_NAMES = {
    MissionStatus.PHASE_IDLE: 'IDLE',
    MissionStatus.PHASE_TRACKING: 'TRACKING',
    MissionStatus.PHASE_LOST_HOLD: 'LOST - HOLD',
    MissionStatus.PHASE_RTL: 'RETURN TO LAUNCH',
    MissionStatus.PHASE_SEARCH: 'SEARCH',
    MissionStatus.PHASE_PURSUE: 'PURSUE',
    MissionStatus.PHASE_CAPTURE: 'CAPTURE',
    MissionStatus.PHASE_DONE: 'DONE - CAPTURED',
    MissionStatus.PHASE_ABORT: 'ABORT',
}

LAW_NAMES = {
    GuidanceCommand.MODE_PN: 'PN',
    GuidanceCommand.MODE_APN: 'APN',
    GuidanceCommand.MODE_PURSUIT: 'PURSUIT',
    GuidanceCommand.MODE_TERMINAL: 'TERMINAL CAPTURE',
    GuidanceCommand.MODE_IDLE: 'IDLE',
}

# One colour per detection class, the same palette as the legend in docs/RVIZ.md.
CLASS_COLOURS = {
    TargetDetection.PERSON: (0.30, 0.75, 1.00),
    TargetDetection.CAR: (1.00, 0.85, 0.20),
    TargetDetection.TRUCK: (1.00, 0.60, 0.20),
    TargetDetection.BICYCLE: (0.70, 0.45, 1.00),
    TargetDetection.UAV: (1.00, 0.25, 0.45),
}


def rgba(r, g, b, a=1.0):
    """Return a ColorRGBA."""
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def point(x, y, z):
    """Return a geometry_msgs Point."""
    return Point(x=float(x), y=float(y), z=float(z))


def quaternion_from_matrix(rot):
    """Return (x, y, z, w) of a proper rotation matrix."""
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((rot[2, 1] - rot[1, 2]) / s, (rot[0, 2] - rot[2, 0]) / s,
                (rot[1, 0] - rot[0, 1]) / s, 0.25 * s)
    if rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        return (0.25 * s, (rot[0, 1] + rot[1, 0]) / s,
                (rot[0, 2] + rot[2, 0]) / s, (rot[2, 1] - rot[1, 2]) / s)
    if rot[1, 1] > rot[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        return ((rot[0, 1] + rot[1, 0]) / s, 0.25 * s,
                (rot[1, 2] + rot[2, 1]) / s, (rot[0, 2] - rot[2, 0]) / s)
    s = 2.0 * math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
    return ((rot[0, 2] + rot[2, 0]) / s, (rot[1, 2] + rot[2, 1]) / s,
            0.25 * s, (rot[1, 0] - rot[0, 1]) / s)


class MissionViz(Node):
    """Turn the pipeline's custom messages into RViz markers and paths."""

    def __init__(self):
        super().__init__('mission_viz_node')
        self.world_frame = self.declare_parameter('world_frame', 'map').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'stereo_left_camera_optical_frame').value
        # The camera of the URDF: 1.047 rad horizontal field of view, 640x480.
        self.camera_hfov = self.declare_parameter('camera_hfov', 1.047).value
        self.camera_aspect = self.declare_parameter('camera_aspect', 480.0 / 640.0).value
        self.frustum_range = self.declare_parameter('frustum_range', 25.0).value
        self.detection_memory = self.declare_parameter('detection_memory', 2.0).value
        self.path_length = int(self.declare_parameter('path_length', 1500).value)
        self.path_min_step = self.declare_parameter('path_min_step', 0.15).value
        self.prediction_horizon = self.declare_parameter('prediction_horizon', 2.0).value
        self.ellipsoid_sigma = self.declare_parameter('ellipsoid_sigma', 2.0).value
        self.rate = self.declare_parameter('rate', 15.0).value
        # Shared with the safety filter and the guidance (safety_params.yaml and
        # intercept_params.yaml are /** files), so the drawing is the real limit.
        self.capture_radius = self.declare_parameter('intercept.capture_radius', 1.5).value
        self.geofence = {
            key: self.declare_parameter('safety.geofence.' + key, default).value
            for key, default in (('x_min', -100.0), ('x_max', 100.0),
                                 ('y_min', -100.0), ('y_max', 100.0))
        }

        self.odom = None
        self.wind = None
        self.mode = None
        self.status = None
        self.guidance = None
        self.guidance_stamp = None
        self.selected = None
        self.selected_stamp = 0.0
        self.tracks = []
        self.tracks_stamp = 0.0
        self.raw_setpoint = None
        self.safe_setpoint = None
        self.detections = deque()

        self.paths = {}
        for name in ('interceptor', 'target_truth', 'target_estimate'):
            self.paths[name] = Path()
            self.paths[name].header.frame_id = self.world_frame

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/target_drone/odom', self.on_target_truth, 10)
        self.create_subscription(Vector3Stamped, '/drone/wind', self.on_wind, 10)
        self.create_subscription(TargetDetection, '/target/detection_3d', self.on_detection, 50)
        self.create_subscription(TargetStateArray, '/tracks', self.on_tracks, 10)
        self.create_subscription(TargetState, '/target/state', self.on_selected, 10)
        self.create_subscription(FlightSetpoint, '/setpoint/raw', self.on_raw, 10)
        self.create_subscription(FlightSetpoint, '/setpoint/safe', self.on_safe, 10)
        self.create_subscription(MissionMode, '/mission/mode', self.on_mode, latched)
        self.create_subscription(MissionStatus, '/mission/status', self.on_status, 10)
        self.create_subscription(GuidanceCommand, '/guidance/command', self.on_guidance, 10)

        self.pub = {name: self.create_publisher(MarkerArray, '/viz/' + name, 10)
                    for name in ('perception', 'estimation', 'guidance', 'hud')}
        self.pub['world'] = self.create_publisher(MarkerArray, '/viz/world', latched)
        self.path_pub = {name: self.create_publisher(Path, '/viz/%s_path' % name, 10)
                         for name in self.paths}

        self.pub['world'].publish(MarkerArray(markers=[self.geofence_marker()]))
        self.create_timer(1.0 / self.rate, self.publish)
        self.get_logger().info('Publishing the RViz layers on /viz/*')

    # --- Inputs ---------------------------------------------------------------

    def now(self):
        """Return the node clock in seconds (sim time under Gazebo)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def append_path(self, name, header, position):
        """Append a pose to a trail when it has moved far enough."""
        path = self.paths[name]
        if path.poses:
            last = path.poses[-1].pose.position
            if math.dist((last.x, last.y, last.z),
                         (position.x, position.y, position.z)) < self.path_min_step:
                return
            # A backwards clock jump (simulation reset) starts a new trail.
            if (header.stamp.sec, header.stamp.nanosec) < (
                    path.poses[-1].header.stamp.sec, path.poses[-1].header.stamp.nanosec):
                path.poses.clear()
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position = position
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
        if len(path.poses) > self.path_length:
            del path.poses[0]

    def on_odom(self, msg):
        self.odom = msg
        self.append_path('interceptor', msg.header, msg.pose.pose.position)

    def on_target_truth(self, msg):
        self.append_path('target_truth', msg.header, msg.pose.pose.position)

    def on_wind(self, msg):
        self.wind = msg.vector

    def on_detection(self, msg):
        if msg.world_position_valid:
            self.detections.append((self.now(), msg.position_world, msg.class_id))

    def on_tracks(self, msg):
        self.tracks = msg.tracks
        self.tracks_stamp = self.now()

    def on_selected(self, msg):
        # track_id -1 is the tracker's "no target".
        self.selected = msg if msg.track_id >= 0 else None
        self.selected_stamp = self.now()
        if self.selected is not None:
            self.append_path('target_estimate', msg.header, msg.position)

    def on_raw(self, msg):
        self.raw_setpoint = msg

    def on_safe(self, msg):
        self.safe_setpoint = msg

    def on_mode(self, msg):
        self.mode = msg

    def on_status(self, msg):
        self.status = msg

    def on_guidance(self, msg):
        self.guidance = msg
        self.guidance_stamp = self.now()

    # --- Markers --------------------------------------------------------------

    def marker(self, ns, marker_id, marker_type, frame=None):
        """Return a marker with the common fields filled in."""
        m = Marker()
        m.header.frame_id = frame or self.world_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        # Short lifetime: a layer whose source stops publishing disappears.
        m.lifetime.nanosec = int(0.5e9)
        return m

    def geofence_marker(self):
        g = self.geofence
        m = self.marker('geofence', 0, Marker.LINE_STRIP)
        m.lifetime.nanosec = 0
        m.scale.x = 0.25
        m.color = rgba(1.0, 0.35, 0.35, 0.8)
        m.points = [point(g['x_min'], g['y_min'], 0.05), point(g['x_max'], g['y_min'], 0.05),
                    point(g['x_max'], g['y_max'], 0.05), point(g['x_min'], g['y_max'], 0.05),
                    point(g['x_min'], g['y_min'], 0.05)]
        return m

    def perception_markers(self):
        markers = []
        now = self.now()
        while self.detections and (now - self.detections[0][0] > self.detection_memory
                                   or now < self.detections[0][0]):
            self.detections.popleft()
        cloud = self.marker('detections', 0, Marker.SPHERE_LIST)
        cloud.scale.x = cloud.scale.y = cloud.scale.z = 0.18
        for stamp, pos, class_id in self.detections:
            fade = 1.0 - (now - stamp) / self.detection_memory
            cloud.points.append(pos)
            cloud.colors.append(rgba(*CLASS_COLOURS.get(class_id, (1.0, 1.0, 1.0)),
                                     0.15 + 0.85 * fade))
        if cloud.points:
            markers.append(cloud)

        # The field of view of the left camera, the one the detector looks through.
        # Optical frame: z forward, x right, y down.
        half_w = math.tan(0.5 * self.camera_hfov) * self.frustum_range
        half_h = half_w * self.camera_aspect
        r = self.frustum_range
        corners = [point(-half_w, -half_h, r), point(half_w, -half_h, r),
                   point(half_w, half_h, r), point(-half_w, half_h, r)]
        origin = point(0.0, 0.0, 0.0)
        edges = self.marker('camera_fov', 0, Marker.LINE_LIST, self.camera_frame)
        edges.frame_locked = True
        edges.header.stamp.sec = 0
        edges.header.stamp.nanosec = 0
        edges.scale.x = 0.04
        edges.color = rgba(0.55, 0.85, 1.0, 0.7)
        for i, corner in enumerate(corners):
            edges.points += [origin, corner, corner, corners[(i + 1) % 4]]
        faces = self.marker('camera_fov', 1, Marker.TRIANGLE_LIST, self.camera_frame)
        faces.frame_locked = True
        faces.header.stamp = edges.header.stamp
        faces.scale.x = faces.scale.y = faces.scale.z = 1.0
        faces.color = rgba(0.55, 0.85, 1.0, 0.06)
        for i, corner in enumerate(corners):
            faces.points += [origin, corner, corners[(i + 1) % 4]]
        markers += [edges, faces]
        return markers

    def ellipsoid(self, track, marker_id, colour):
        """Return the position covariance of a track as an ellipsoid."""
        cov = np.array(track.position_covariance, dtype=float).reshape(3, 3)
        cov = 0.5 * (cov + cov.T)
        values, vectors = np.linalg.eigh(cov)
        if np.linalg.det(vectors) < 0.0:
            vectors[:, 0] = -vectors[:, 0]
        m = self.marker('covariance', marker_id, Marker.SPHERE)
        m.pose.position = track.position
        qx, qy, qz, qw = quaternion_from_matrix(vectors)
        m.pose.orientation.x, m.pose.orientation.y = float(qx), float(qy)
        m.pose.orientation.z, m.pose.orientation.w = float(qz), float(qw)
        axes = 2.0 * self.ellipsoid_sigma * np.sqrt(np.clip(values, 1e-6, None))
        m.scale.x, m.scale.y, m.scale.z = (float(max(a, 0.05)) for a in axes)
        m.color = colour
        return m

    def estimation_markers(self):
        markers = []
        selected_id = self.selected.track_id if self.selected is not None else None
        for i, track in enumerate(self.tracks):
            if track.track_id == selected_id:
                colour = rgba(1.0, 0.55, 0.0, 0.35)
            elif track.confirmed and track.is_valid:
                colour = rgba(0.2, 0.9, 0.4, 0.25)
            else:
                colour = rgba(0.7, 0.7, 0.7, 0.15)
            markers.append(self.ellipsoid(track, i, colour))

        if self.selected is not None:
            # Where the tracker thinks the target will be (constant acceleration),
            # drawn dashed: it is a prediction, not a measurement.
            s = self.selected
            p = np.array([s.position.x, s.position.y, s.position.z])
            v = np.array([s.velocity.x, s.velocity.y, s.velocity.z])
            a = np.array([s.acceleration.x, s.acceleration.y, s.acceleration.z])
            steps = 20
            samples = [p + v * t + 0.5 * a * t * t
                       for t in np.linspace(0.0, self.prediction_horizon, steps + 1)]
            dashes = self.marker('prediction', 0, Marker.LINE_LIST)
            dashes.scale.x = 0.07
            dashes.color = rgba(1.0, 0.55, 0.0, 0.9)
            for k in range(0, steps, 2):
                dashes.points += [point(*samples[k]), point(*samples[k + 1])]
            end = self.marker('prediction', 1, Marker.SPHERE)
            end.pose.position = point(*samples[-1])
            end.scale.x = end.scale.y = end.scale.z = 0.3
            end.color = rgba(1.0, 0.55, 0.0, 0.6)
            markers += [dashes, end]
        return markers

    def guidance_markers(self):
        markers = []
        if self.odom is None:
            return markers
        drone = self.odom.pose.pose.position

        # Altitude drop lines: a 3D view hides height, a line to the ground shows it.
        drops = self.marker('altitude', 0, Marker.LINE_LIST)
        drops.scale.x = 0.03
        drops.color = rgba(0.8, 0.8, 0.8, 0.45)
        drops.points += [drone, point(drone.x, drone.y, 0.0)]
        if self.selected is not None:
            t = self.selected.position
            drops.points += [t, point(t.x, t.y, 0.0)]
        markers.append(drops)

        if self.selected is not None:
            t = self.selected.position
            rng = math.dist((drone.x, drone.y, drone.z), (t.x, t.y, t.z))
            line = self.marker('range', 0, Marker.LINE_LIST)
            line.scale.x = 0.05
            line.color = rgba(1.0, 1.0, 1.0, 0.6)
            line.points = [drone, t]
            label = self.marker('range', 1, Marker.TEXT_VIEW_FACING)
            label.pose.position = point(0.5 * (drone.x + t.x), 0.5 * (drone.y + t.y),
                                        0.5 * (drone.z + t.z) + 0.5)
            label.scale.z = 0.6
            label.color = rgba(1.0, 1.0, 1.0, 0.9)
            label.text = '%.1f m' % rng
            markers += [line, label]
            if self.mode is not None and self.mode.mode == MissionMode.MODE_INTERCEPT:
                envelope = self.marker('capture_envelope', 0, Marker.SPHERE)
                envelope.pose.position = t
                envelope.scale.x = envelope.scale.y = envelope.scale.z = 2.0 * self.capture_radius
                envelope.color = rgba(1.0, 0.2, 0.3, 0.12)
                markers.append(envelope)

        if self.raw_setpoint is not None:
            # The planner's request, ghosted. The safety filter's own arrow
            # (/safety/keepouts) is the one the controller flies: where the two
            # differ, the filter stepped in.
            v = self.raw_setpoint.velocity
            arrow = self.marker('planner_command', 0, Marker.ARROW)
            arrow.points = [drone, point(drone.x + v.x, drone.y + v.y, drone.z + v.z)]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.07, 0.2, 0.25
            arrow.color = rgba(0.3, 0.6, 1.0, 0.55)
            markers.append(arrow)
        return markers

    def hud_markers(self):
        if self.odom is None:
            return []
        drone = self.odom.pose.pose.position
        twist = self.odom.twist.twist.linear
        speed = math.sqrt(twist.x ** 2 + twist.y ** 2 + twist.z ** 2)

        mode = MODE_NAMES.get(self.mode.mode, '?') if self.mode is not None else 'NO MISSION'
        title = mode
        if self.mode is not None and self.mode.mode == MissionMode.MODE_INTERCEPT:
            title += '  ARMED' if self.mode.armed else '  DISARMED'

        lines = []
        if self.status is not None:
            lines.append('phase  %s' % PHASE_NAMES.get(self.status.phase, str(self.status.phase)))
        if self.selected is not None:
            s = self.selected
            target_speed = math.sqrt(s.velocity.x ** 2 + s.velocity.y ** 2 + s.velocity.z ** 2)
            lines.append('target  #%d %s  %.1f m/s%s' % (
                s.track_id, s.class_name or '?', target_speed,
                '' if s.is_valid else '  (coasting)'))
        else:
            lines.append('target  none')
        fresh = self.guidance_stamp is not None and self.now() - self.guidance_stamp < 0.5
        if (fresh and self.mode is not None
                and self.mode.mode == MissionMode.MODE_INTERCEPT):
            g = self.guidance
            lines.append('%s  N=%.1f' % (LAW_NAMES.get(g.mode, '?'), g.navigation_constant))
            lines.append('Vc %.1f m/s   t_go %s   ZEM %.2f m' % (
                g.closing_velocity,
                '%.1f s' % g.time_to_go if math.isfinite(g.time_to_go) else '-',
                g.miss_distance))
        lines.append('speed %.1f m/s   alt %.1f m' % (speed, drone.z))
        if self.wind is not None:
            wind = math.hypot(self.wind.x, self.wind.y)
            if wind > 0.05:
                lines.append('wind %.1f m/s' % wind)

        # TEXT_VIEW_FACING is centred on its position, so the blocks are
        # stacked upwards from base_z: the safety line, the body, the title.
        base_z = drone.z + 1.6
        line_height = 0.42
        body_height = 1.2 * line_height * len(lines)
        body = self.marker('hud', 1, Marker.TEXT_VIEW_FACING)
        body.scale.z = line_height
        body.color = rgba(0.95, 0.95, 0.95, 0.95)
        body.text = '\n'.join(lines)
        body.pose.position = point(drone.x, drone.y, base_z + 0.5 * body_height)
        head = self.marker('hud', 0, Marker.TEXT_VIEW_FACING)
        head.scale.z = 0.7
        head.color = rgba(0.35, 0.85, 1.0, 1.0)
        head.text = title
        head.pose.position = point(drone.x, drone.y, base_z + body_height + 0.5)
        markers = [head, body]

        if self.status is not None and (self.status.abort_active or self.status.safety_active):
            safety = self.marker('hud', 2, Marker.TEXT_VIEW_FACING)
            safety.scale.z = 0.45
            if self.status.abort_active:
                safety.color = rgba(1.0, 0.2, 0.2, 1.0)
                safety.text = 'ABORT: ' + self.status.reason
            else:
                safety.color = rgba(1.0, 0.65, 0.1, 1.0)
                safety.text = 'SAFETY FILTER: ' + (self.status.reason or 'active')
            safety.pose.position = point(drone.x, drone.y, base_z - 0.2)
            markers.append(safety)
        return markers

    # --- Output ---------------------------------------------------------------

    def publish(self):
        # A stopped tracker leaves nothing behind on the screen.
        now = self.now()
        if abs(now - self.selected_stamp) > 1.0:
            self.selected = None
        if abs(now - self.tracks_stamp) > 1.0:
            self.tracks = []
        self.pub['perception'].publish(MarkerArray(markers=self.perception_markers()))
        self.pub['estimation'].publish(MarkerArray(markers=self.estimation_markers()))
        self.pub['guidance'].publish(MarkerArray(markers=self.guidance_markers()))
        self.pub['hud'].publish(MarkerArray(markers=self.hud_markers()))
        for name, path in self.paths.items():
            path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub[name].publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = MissionViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
