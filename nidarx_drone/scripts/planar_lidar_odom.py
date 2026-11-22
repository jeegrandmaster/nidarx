#!/usr/bin/env python3
"""Planar point-to-line lidar odometry for the indoor quadcopter."""

from collections import deque
import math

from lidar_runtime import quaternion_matrix, run_node, stamp_ns
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def level_scan(scan, attitude, mount):
    """Return ordered endpoints in a horizontal, body-heading-aligned frame."""
    world_from_body = quaternion_matrix(attitude)
    body_from_laser = quaternion_matrix(mount.rotation)
    if world_from_body is None or body_from_laser is None:
        return None
    # At large tilt, floor and ceiling returns can dominate this 2D model.
    if world_from_body[2, 2] < math.cos(math.radians(12)):
        return None
    if not all(math.isfinite(v) for v in (
            scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)):
        return None
    if scan.angle_increment == 0 or scan.range_min < 0 or scan.range_max <= scan.range_min:
        return None

    # Keep scan order (including missing returns) for adjacent surface normals.
    stride = max(1, math.ceil(len(scan.ranges) / 480))
    ranges = np.array(scan.ranges[::stride], dtype=float)
    ranges[(ranges < scan.range_min) | (ranges >= scan.range_max)] = np.nan
    angles = scan.angle_min + np.arange(len(ranges)) * stride * scan.angle_increment
    ray_directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
    translation = mount.translation
    body_points = ranges[:, None] * (ray_directions @ body_from_laser.T)
    body_points += [translation.x, translation.y, translation.z]
    yaw = math.atan2(world_from_body[1, 0], world_from_body[0, 0])
    projection = rotation(-yaw) @ world_from_body[:2, :]
    return body_points @ projection.T


def reference_surfaces(points):
    """Estimate normals from adjacent hits and omit gaps and sharp corners."""
    centers = points[1:-1]
    before, after = centers - points[:-2], points[2:] - centers
    before_len = np.linalg.norm(before, axis=1)
    after_len = np.linalg.norm(after, axis=1)
    good = ((before_len > 0.004) & (after_len > 0.004) &
            (before_len < 0.5) & (after_len < 0.5) &
            ((before * after).sum(axis=1) > 0.85 * before_len * after_len))
    tangent = (before + after)[good]
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    return centers[good], np.column_stack((-tangent[:, 1], tangent[:, 0]))


def register(points, reference, initial_pose):
    """Register a scan to fixed lines in odom; reject weakly observed motion."""
    targets, normals = reference
    if len(points) < 30 or len(targets) < 30:
        return None
    pose = initial_pose.copy()
    for _ in range(12):
        relative = points @ rotation(pose[2]).T
        projected = relative + pose[:2]
        dx = projected[:, None, 0] - targets[:, 0]
        dy = projected[:, None, 1] - targets[:, 1]
        distances_sq = dx*dx + dy*dy
        nearest = distances_sq.argmin(axis=1)
        close = distances_sq[np.arange(len(points)), nearest] < 0.50**2
        if np.count_nonzero(close) < max(30, len(points)*0.6):
            return None
        n = normals[nearest[close]]
        offsets = projected[close] - targets[nearest[close]]
        errors = (offsets * n).sum(axis=1)
        lever = relative[close]
        heading = lever[:, 0]*n[:, 1] - lever[:, 1]*n[:, 0]
        jacobian = np.column_stack((n, heading))
        # Huber weights reduce influence from moved objects / different scan heights.
        weights = np.sqrt(0.04 / np.maximum(0.04, np.abs(errors)))
        weighted = weights[:, None] * jacobian
        # Scale the yaw column by typical point distance for a dimensionless
        # observability test. Parallel walls lose translation, a circle loses yaw.
        scale = max(0.3, np.median(np.linalg.norm(lever, axis=1)))
        singular = np.linalg.svd(weighted / [1, 1, scale], compute_uv=False)
        if singular[-1] < 0.12 * singular[0]:
            return None
        step = np.linalg.lstsq(weighted, -weights * errors, rcond=None)[0]
        pose += step
        if np.linalg.norm(step[:2]) < 0.001 and abs(step[2]) < 0.0005:
            if np.count_nonzero(np.abs(errors) < 0.075) >= max(30, len(points)*0.6):
                pose[2] = wrap_angle(pose[2])
                return pose
            return None
    return None


class ScanOdometry:
    """Keep a fixed reference while hovering so noise doesn't integrate as drift."""

    def __init__(self):
        self.pose = np.zeros(3)
        self.anchor = self.pose.copy()
        self.reference = None

    def update(self, scan_points):
        points = scan_points[np.isfinite(scan_points).all(axis=1)]
        if len(points) < 30:
            return None
        if self.reference is None:
            first_reference = reference_surfaces(scan_points)
            if register(points, first_reference, self.pose) is None:
                return None
            self.reference = first_reference
            return self.pose.copy()

        pose = register(points, self.reference, self.pose)
        if pose is None:
            return None
        if (np.linalg.norm(pose[:2] - self.pose[:2]) > 0.5 or
                abs(wrap_angle(pose[2] - self.pose[2])) > math.radians(18)):
            return None
        if (np.linalg.norm(pose[:2] - self.anchor[:2]) > 0.55 or
                abs(wrap_angle(pose[2] - self.anchor[2])) > math.radians(15)):
            new_reference = reference_surfaces(scan_points @ rotation(pose[2]).T + pose[:2])
            if register(points, new_reference, pose) is not None:
                self.reference = new_reference
                self.anchor = pose.copy()
        self.pose = pose
        return pose.copy()


class PlanarLidarOdometry(Node):

    def __init__(self):
        super().__init__('planar_lidar_odometry')
        self.estimator = ScanOdometry()
        self.attitudes = deque(maxlen=500)
        self.last_scan_ns = None
        self.previous = None
        self.last_clock_ns = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Odometry, '/nidarx_drone/lidar/odom', 10)
        self.create_subscription(
            Imu, '/nidarx_drone/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/nidarx_drone/rplidar/data', self.on_scan, qos_profile_sensor_data)

    def on_imu(self, msg):
        if msg.orientation_covariance[0] >= 0 and msg.header.frame_id == 'imu_link':
            self.attitudes.append((stamp_ns(msg), msg.orientation))

    def on_scan(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.estimator = ScanOdometry()
            self.attitudes.clear()
            self.last_scan_ns = self.previous = None
        self.last_clock_ns = now_ns
        scan_ns = stamp_ns(msg)
        if self.last_scan_ns is not None and scan_ns <= self.last_scan_ns:
            return
        age_ns = now_ns - scan_ns
        if not self.attitudes or not msg.header.frame_id:
            return
        # Gazebo's /clock may trail sensor stamps by one 10 Hz clock tick.
        if not -120_000_000 <= age_ns <= 500_000_000:
            return
        self.last_scan_ns = scan_ns
        imu_ns, attitude = min(self.attitudes, key=lambda item: abs(item[0] - scan_ns))
        if abs(imu_ns - scan_ns) > 80_000_000:
            return
        try:
            mount = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
        except TransformException:
            return
        scan = level_scan(msg, attitude, mount.transform)
        pose = self.estimator.update(scan) if scan is not None else None
        if pose is None:
            return

        msg_out = Odometry()
        msg_out.header.frame_id = 'lidar_odom'
        msg_out.header.stamp = msg.header.stamp
        msg_out.child_frame_id = 'base_link'
        p = msg_out.pose.pose
        p.position.x, p.position.y = map(float, pose[:2])
        p.orientation.z = math.sin(pose[2]/2)
        p.orientation.w = math.cos(pose[2]/2)
        for i, variance in enumerate((0.04**2, 0.04**2, 1e6, 1e6, 1e6, 0.03**2)):
            msg_out.pose.covariance[7*i] = variance
            msg_out.twist.covariance[7*i] = 1e6
        if self.previous is not None:
            last_ns, last_pose = self.previous
            dt = (scan_ns - last_ns) * 1e-9
            if 0 < dt <= 0.5:
                velocity = rotation(-pose[2]) @ ((pose[:2] - last_pose[:2])/dt)
                msg_out.twist.twist.linear.x, msg_out.twist.twist.linear.y = map(float, velocity)
                dyaw = pose[2] - last_pose[2]
                msg_out.twist.twist.angular.z = wrap_angle(dyaw)/dt
                for index in (0, 7, 35):
                    msg_out.twist.covariance[index] = 0.1
        self.previous = scan_ns, pose
        self.pub.publish(msg_out)


def main(args=None):
    run_node(PlanarLidarOdometry, args)


if __name__ == '__main__':
    main()
