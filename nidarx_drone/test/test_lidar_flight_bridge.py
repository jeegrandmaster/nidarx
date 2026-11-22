"""Exercise the sensor and command boundaries without opening a flight link."""

import asyncio
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from lidar_flight_bridge import LidarFlightBridge, MAV, NS   # noqa: E402, I100


class FlightBridgeTest(unittest.TestCase):

    def setUp(self):
        self.node = SimpleNamespace(
            target=1.5, pending=None, heartbeat=Mock(), link=Mock(),
            height=0.19, height_stamp=NS, range_stamp=None, pose_stamp=None,
            get_clock=Mock(), last_heartbeat=time.monotonic(), executor=None)
        self.node.get_clock.return_value.now.return_value.nanoseconds = NS
        self.node.heartbeat.get_srcSystem.return_value = 1
        self.node.heartbeat.custom_mode = MAV.COPTER_MODE_GUIDED
        self.node.heartbeat.base_mode = MAV.MAV_MODE_FLAG_SAFETY_ARMED
        self.scan = LaserScan()
        self.scan.header.frame_id = 'downward_lidar_link'
        self.scan.header.stamp.sec = 1
        self.scan.range_min = 0.05
        self.scan.range_max = 10.0
        self.scan.ranges = [1.455]
        self.odom = Odometry()
        self.odom.header.frame_id = 'lidar_odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.header.stamp.sec = 1
        pose = self.odom.pose.pose
        pose.position.x, pose.position.y = 1.0, 2.0
        pose.orientation.z = math.sin(0.4)
        pose.orientation.w = math.cos(0.4)

    def test_single_valid_range_goes_to_downward_sensor_in_cm(self):
        for values in ([float('nan')], [float('inf')], [], [1., 2.], [10.], [12.]):
            self.scan.ranges = values
            LidarFlightBridge.on_range(self.node, self.scan)
        self.node.link.mav.distance_sensor_send.assert_not_called()
        self.scan.ranges = [1.46]
        self.scan.header.stamp.nanosec = 15_000_000
        for _ in range(2):
            LidarFlightBridge.on_range(self.node, self.scan)
        self.node.link.mav.distance_sensor_send.assert_called_once_with(
            1015, 5, 1000, 146, MAV.MAV_DISTANCE_SENSOR_LASER,
            0, MAV.MAV_SENSOR_ROTATION_PITCH_270, 255)

    def test_planar_flight_axes_and_old_packet(self):
        LidarFlightBridge.on_odom(self.node, self.odom)
        args = self.node.link.mav.vision_position_estimate_send.call_args.args
        self.assertEqual(args[:-1], (1_000_000, 1.0, -2.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(args[-1], -0.8)
        self.odom.header.stamp.sec = 0
        LidarFlightBridge.on_odom(self.node, self.odom)
        self.node.link.mav.vision_position_estimate_send.assert_called_once()

    def test_takeoff_requires_fresh_pose_range_and_pilot_arming(self):
        def takeoff():
            return asyncio.run(LidarFlightBridge.takeoff(
                self.node, Trigger.Request(), Trigger.Response()))
        self.assertFalse(takeoff().success)
        self.node.pose_stamp = self.node.range_stamp = NS
        self.node.heartbeat.base_mode = 0
        self.assertFalse(takeoff().success)
        self.node.link.mav.command_int_send.assert_not_called()

    def test_takeoff_sends_height_above_home_and_waits_for_ack(self):
        self.node.pose_stamp = self.node.range_stamp = NS

        def ack(*args):
            self.node.pending.set_result((True, 'ack'))
        self.node.link.mav.command_int_send.side_effect = ack
        response = asyncio.run(LidarFlightBridge.takeoff(
            self.node, Trigger.Request(), Trigger.Response()))
        self.assertTrue(response.success)
        args = self.node.link.mav.command_int_send.call_args.args
        self.assertEqual(args[:4], (
            1, MAV.MAV_COMP_ID_AUTOPILOT1, MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            MAV.MAV_CMD_NAV_TAKEOFF))
        self.assertAlmostEqual(args[-1], 1.31)


if __name__ == '__main__':
    unittest.main()
