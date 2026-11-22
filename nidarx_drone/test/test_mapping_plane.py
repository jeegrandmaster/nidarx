"""Check altitude gating and projection with synthetic sensor messages."""

from collections import deque
import math
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Quaternion
import numpy as np
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from mapping_plane import (  # noqa: E402, I100
    body_up, MappingPlane, PlaneGate, recent_at, scan_in_height_band, with_lidar_heading)


def test_mapping_requires_continuous_level_dwell_and_fresh_data():
    gate = PlaneGate(1.5)
    tick = 50_000_000
    for i in range(40):
        assert not gate.update(i * tick, 0.19, 1.0)
    for i in range(40, 60):
        assert not gate.update(i * tick, 1.5, 1.0)
    assert gate.update(60 * tick, 1.5, 1.0)
    assert gate.update(61 * tick, 1.5, math.cos(math.radians(6)))
    assert not gate.update(62 * tick, 1.5, math.cos(math.radians(10)))
    for i in range(63, 89):
        assert not gate.update(i * tick, 1.5, math.cos(math.radians(5)))
    assert not gate.update(100 * tick, 1.5, 1.0)
    for i in range(101, 120):
        assert not gate.update(i * tick, 1.5, 1.0)
    assert gate.update(120 * tick, 1.5, 1.0)
    assert not gate.update(121 * tick, 1.2, 1.0)


def test_tilted_moving_scan_only_maps_hits_in_the_optical_plane():
    roll = math.radians(6)
    up = body_up(Quaternion(x=math.sin(roll/2), w=math.cos(roll/2)))
    scan = LaserScan(angle_min=0., angle_increment=math.pi/180, range_min=0.15, range_max=12.)
    scan.ranges = [5.] * 360
    scan.ranges[180] = math.inf
    # A scanner whose +X axis points along body +Y.
    mount = [0., 0., 0.05], np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    result = scan_in_height_band(scan, up, mount, height=1.5, target=1.5)
    assert result is not None
    assert result.ranges[90] == result.ranges[270] == 5.
    assert math.isnan(result.ranges[0]) and math.isnan(result.ranges[180])
    assert len(result.ranges) == 360 and scan.ranges[0] == 5.
    scan.ranges = [math.nan] * 360
    assert scan_in_height_band(scan, up, mount, height=1.5, target=1.5) is None


def test_downward_slant_lever_arm_and_invalid_measurement():
    roll = math.radians(15)
    imu = Quaternion(x=math.sin(roll/2), w=math.cos(roll/2))
    mount = [0., 0., -0.045], np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    node = SimpleNamespace(
        sensor_mount=lambda frame: mount,
        attitudes=deque([(0, imu)]), heights=deque(), gate=PlaneGate(1.5), height_pub=Mock())
    scan = LaserScan(range_min=0.05, range_max=10.0)
    scan.header.frame_id = 'downward_lidar_link'
    scan.ranges = [1.5 / math.cos(roll) - 0.045]
    MappingPlane.on_down(node, scan)
    assert abs(node.heights[-1][1][0] - 1.5) < 1e-6
    assert node.height_pub.publish.call_count == 1
    scan.header.stamp.nanosec = 10_000_000
    scan.ranges = [math.inf]
    MappingPlane.on_down(node, scan)
    assert recent_at(node.heights, 10_000_001) is None
    assert node.height_pub.publish.call_count == 1
    assert recent_at(node.attitudes, 120_000_000) is None


def test_scan_heading_replaces_imu_yaw_and_preserves_tilt():
    a, b = 0.2, 0.4  # half angles of roll and initial yaw
    imu = Quaternion(x=math.sin(a)*math.cos(b), y=math.sin(a)*math.sin(b),
                     z=math.cos(a)*math.sin(b), w=math.cos(a)*math.cos(b))
    heading = -0.55
    q = with_lidar_heading(imu, Quaternion(z=math.sin(heading/2), w=math.cos(heading/2)))
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    assert math.isclose(yaw, heading)
    assert all(abs(a-b) < 1e-12 for a, b in zip(body_up(imu), body_up(q)))
