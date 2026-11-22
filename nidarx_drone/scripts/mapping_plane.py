#!/usr/bin/env python3
"""Build the measured body TF and admit scans from the selected flight plane."""

from collections import deque
from copy import copy
import math

from geometry_msgs.msg import PointStamped, Quaternion, TransformStamped
from lidar_runtime import pilot_altitude, quaternion_matrix, run_node, stamp_ns
from message_filters import Subscriber, TimeSynchronizer
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


HEIGHT_BAND = 0.10
SETTLE_UP_Z = math.cos(math.radians(3))
MAPPING_UP_Z = math.cos(math.radians(8))


def recent_at(history, stamp):
    # Causal lookup lets delayed odometry use the acquisition-time samples.
    for sample in reversed(history):
        if sample[0] <= stamp:
            return sample[1] if stamp - sample[0] <= 100_000_000 else None
    return None


def body_up(q):
    """World vertical as a unit vector in the body frame."""
    return (2 * (q.x * q.z - q.w * q.y),
            2 * (q.y * q.z + q.w * q.x),
            1 - 2 * (q.x * q.x + q.y * q.y))


def with_lidar_heading(imu, planar):
    """Rotate about world Z to retain measured roll/pitch and replace yaw."""
    imu_yaw = math.atan2(2 * (imu.w * imu.z + imu.x * imu.y),
                         1 - 2 * (imu.y * imu.y + imu.z * imu.z))
    half_delta = math.atan2(planar.z, planar.w) - imu_yaw / 2
    c, s = math.cos(half_delta), math.sin(half_delta)
    return Quaternion(x=c * imu.x - s * imu.y, y=c * imu.y + s * imu.x,
                      z=c * imu.z + s * imu.w, w=c * imu.w - s * imu.z)


class PlaneGate:
    """Require a continuous second of level, in-band range measurements."""

    def __init__(self, target):
        self.target = target
        self.start = None
        self.last = None
        self.settled = False

    def update(self, stamp, height, up_z):
        continuous = self.last is not None and 0 < stamp - self.last <= 100_000_000
        self.last = stamp
        if not continuous:
            self.settled = False
            self.start = None
        min_up_z = MAPPING_UP_Z if self.settled else SETTLE_UP_Z
        if not (abs(height - self.target) <= HEIGHT_BAND and up_z >= min_up_z):
            self.start = None
            self.settled = False
            return False
        if self.start is None:
            self.start = stamp
        self.settled = stamp - self.start >= 1_000_000_000
        return self.settled


def scan_in_height_band(scan, up, mount, height, target):
    """Keep measured hits that lie within 10 cm of the selected optical plane."""
    origin, rotation = mount
    sensor_height = height + np.dot(up, origin)
    plane_height = target + origin[2]
    up_in_sensor = up @ rotation
    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    beam_z = up_in_sensor[0] * np.cos(angles) + up_in_sensor[1] * np.sin(angles)
    ranges = np.array(scan.ranges, dtype=float)
    ranges[(ranges < scan.range_min) | (ranges >= scan.range_max)] = np.nan
    hits_in_band = np.abs(sensor_height + ranges * beam_z - plane_height) <= HEIGHT_BAND
    if np.count_nonzero(hits_in_band) < 20:
        return None
    if hits_in_band.all():
        return scan
    # NaN marks an unavailable beam. An off-plane ray must not clear cells.
    filtered = copy(scan)
    ranges[~hits_in_band] = np.nan
    filtered.ranges = ranges.tolist()
    return filtered


class MappingPlane(Node):

    def __init__(self):
        super().__init__('mapping_plane')
        self.gate = PlaneGate(pilot_altitude(self))
        self.attitudes = deque(maxlen=2048)
        self.heights = deque(maxlen=128)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mounts = {}
        self.body_tf = TransformBroadcaster(self)
        self.height_pub = self.create_publisher(PointStamped, '/nidarx_drone/lidar/height', 5)
        self.scan_pub = self.create_publisher(LaserScan, '/nidarx_drone/mapping_scan', 5)
        self.create_subscription(
            Imu, '/nidarx_drone/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/nidarx_drone/downward_lidar/data', self.on_down,
                                 qos_profile_sensor_data)
        scan = Subscriber(self, LaserScan, '/nidarx_drone/rplidar/data', qos_profile_sensor_data)
        pose = Subscriber(self, Odometry, '/nidarx_drone/lidar/odom', qos_profile_sensor_data)
        self.sync = TimeSynchronizer([scan, pose], 10)
        self.sync.registerCallback(self.on_matched_scan)

    def on_imu(self, msg):
        stamp = stamp_ns(msg)
        if self.attitudes and stamp < self.attitudes[-1][0]:
            self.attitudes.clear()
        q = msg.orientation
        valid = (msg.header.frame_id == 'imu_link' and msg.orientation_covariance[0] != -1
                 and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < 0.001)
        self.attitudes.append((stamp, q if valid else None))

    def sensor_mount(self, frame):
        if frame not in self.mounts:
            try:
                mount = self.tf_buffer.lookup_transform('base_link', frame, Time()).transform
            except TransformException:
                return None
            p = mount.translation
            rotation = quaternion_matrix(mount.rotation)
            if rotation is None:
                return None
            self.mounts[frame] = np.array([p.x, p.y, p.z]), rotation
        return self.mounts[frame]

    def on_down(self, msg):
        stamp = stamp_ns(msg)
        if self.heights and stamp < self.heights[-1][0]:
            self.heights.clear()
        attitude = recent_at(self.attitudes, stamp)
        mount = self.sensor_mount('downward_lidar_link')
        height = math.nan
        up_z = 0.0
        if (attitude is not None and mount is not None
                and msg.header.frame_id == 'downward_lidar_link' and len(msg.ranges) == 1
                and math.isfinite(msg.ranges[0])
                and msg.range_min <= msg.ranges[0] < msg.range_max):
            up = body_up(attitude)
            up_z = up[2]
            origin, rotation = mount
            projection = np.dot(up, rotation[:, 0])  # Single-beam LaserScan along sensor +X.
            if projection < -0.7:
                height = -np.dot(up, origin) - msg.ranges[0] * projection

        if not math.isfinite(height) or height <= 0:
            self.heights.append((stamp, None))
            self.gate.update(stamp, math.inf, 0.0)
            return

        ready = self.gate.update(stamp, height, up_z)
        self.heights.append((stamp, (height, ready)))
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'lidar_odom'
        out.point.z = height
        self.height_pub.publish(out)

    def on_matched_scan(self, scan, odom):
        stamp = stamp_ns(scan)
        attitude = recent_at(self.attitudes, stamp)
        sample = recent_at(self.heights, stamp)
        if sample is None or attitude is None:
            return
        height, ready = sample
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = height
        tf.transform.rotation = with_lidar_heading(attitude, odom.pose.pose.orientation)
        self.body_tf.sendTransform(tf)
        up = body_up(attitude)
        if not ready or up[2] < MAPPING_UP_Z or scan.header.frame_id != 'rplidar_link':
            return
        mount = self.sensor_mount(scan.header.frame_id)
        if mount is None:
            return
        filtered = scan_in_height_band(scan, up, mount, height, self.gate.target)
        if filtered is not None:
            self.scan_pub.publish(filtered)


def main(args=None):
    run_node(MappingPlane, args)


if __name__ == '__main__':
    main()
