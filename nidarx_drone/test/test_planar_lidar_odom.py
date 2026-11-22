"""Deterministic laser scans of the four walls of indoor.world."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped
import numpy as np
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from lidar_runtime import quaternion_matrix  # noqa: E402, I100
from planar_lidar_odom import (  # noqa: E402, I100
    level_scan, PlanarLidarOdometry, ScanOdometry)


def attitude(roll, pitch, yaw):
    sr, sp, sy = np.sin(np.array([roll, pitch, yaw]) / 2)
    cr, cp, cy = np.cos(np.array([roll, pitch, yaw]) / 2)
    return Quaternion(
        x=sr*cp*cy - cr*sp*sy, y=cr*sp*cy + sr*cp*sy,
        z=cr*cp*sy - sr*sp*cy, w=cr*cp*cy + sr*sp*sy)


def room_observations(x=0.0, y=0.0, yaw=0.0, roll=0.0, pitch=0.0, rng=None):
    scan = LaserScan(angle_min=0.0, angle_increment=2*math.pi / 360,
                     range_min=0.15, range_max=12.0)
    mount = Transform()
    mount.rotation.w = 1.0
    mount.translation.z = 0.05
    q = attitude(roll, pitch, yaw)
    matrix = quaternion_matrix(q)
    origin = matrix @ [0, 0, mount.translation.z] + [x, y, 1.5]
    angles = scan.angle_min + np.arange(360) * scan.angle_increment
    rays = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(360))) @ matrix.T
    with np.errstate(divide='ignore'):
        distance = np.minimum(
            (np.copysign(4.925, rays[:, 0]) - origin[0]) / rays[:, 0],
            (np.copysign(4.925, rays[:, 1]) - origin[1]) / rays[:, 1])
    if rng is not None:
        distance += rng.normal(scale=0.008, size=len(distance))
        distance[rng.choice(len(distance), 30, replace=False)] = np.inf
    scan.ranges = distance.astype('float32').tolist()
    return scan, q, mount


def room_scan(*args, **kwargs):
    return level_scan(*room_observations(*args, **kwargs))


class PlanarOdometryTest(unittest.TestCase):

    def test_motion_across_keyframes_with_attitude_and_missing_returns(self):
        odometry = ScanOdometry()
        rng = np.random.default_rng(808)
        for t in np.linspace(0, 1, 21):
            expected = np.array([1.4*t, -0.85*t, 0.6*t])
            points = room_scan(*expected, roll=0.07*t, pitch=-0.1*t, rng=rng)
            pose = odometry.update(points)
            self.assertIsNotNone(pose)
            np.testing.assert_allclose(pose, expected, atol=0.025)

    def test_stationary_scan_has_no_accumulated_drift(self):
        odometry = ScanOdometry()
        rng = np.random.default_rng(15)
        for _ in range(40):
            pose = odometry.update(room_scan(rng=rng))
            np.testing.assert_allclose(pose, np.zeros(3), atol=0.01)

    def test_unobservable_or_corrupt_scans_do_not_seed_or_overwrite_position(self):
        odometry = ScanOdometry()
        angles = np.linspace(0, 2*math.pi, 360)
        circle = np.column_stack((5*np.cos(angles), 5*np.sin(angles)))
        line = np.column_stack((np.linspace(-4.0, 4.0, 100), np.full(100, 4.925)))
        invalid = np.full((360, 2), np.nan)
        for scan in (line, circle, invalid):
            self.assertIsNone(odometry.update(scan))
        odometry.update(room_scan())
        odometry.update(room_scan(x=0.21))
        for scan in (line, invalid):
            self.assertIsNone(odometry.update(scan))
        np.testing.assert_allclose(odometry.update(room_scan(x=0.27)), [0.27, 0, 0], atol=0.01)

    def test_excess_tilt_skips_scan(self):
        self.assertIsNone(room_scan(roll=math.radians(15)))

    def test_sensor_stamps_can_lead_clock_and_recover_after_bad_stamp(self):
        scan, attitude_q, mount = room_observations()
        scan.header.frame_id = 'rplidar_link'
        now = 100_000_000_000
        clock = Mock()
        clock.now.return_value.nanoseconds = now
        node = SimpleNamespace(attitudes=[], estimator=ScanOdometry(), last_scan_ns=None,
                               last_clock_ns=None, previous=None, get_clock=lambda: clock,
                               tf_buffer=Mock(), pub=Mock())
        node.tf_buffer.lookup_transform.return_value = TransformStamped(transform=mount)
        for offset in (95_000_000, 500_000_000, 110_000_000):
            node.attitudes.append((now + offset, attitude_q))
            scan.header.stamp = Time(sec=100, nanosec=offset)
            PlanarLidarOdometry.on_scan(node, scan)
        calls = node.pub.publish.call_args_list
        self.assertEqual([call.args[0].header.stamp.nanosec for call in calls],
                         [95_000_000, 110_000_000])
