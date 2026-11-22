#!/usr/bin/env python3
"""Feed measured lidar observations to the indoor ArduCopter SITL instance."""

import math
import time

from geometry_msgs.msg import PointStamped
from lidar_runtime import pilot_altitude, run_node, stamp_ns
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


MAV = mavutil.mavlink
NS = 1_000_000_000


def fresh(stamp, now, max_age=0.5):
    # A Gazebo sensor packet can arrive just before its matching /clock tick.
    return stamp is not None and -0.1 * NS <= now - stamp <= max_age * NS


def range_cm(scan):
    if scan.header.frame_id != 'downward_lidar_link' or len(scan.ranges) != 1:
        return None
    low, high, distance = scan.range_min, scan.range_max, scan.ranges[0]
    if not all(map(math.isfinite, (low, high, distance))):
        return None
    if not (0 <= low < high <= 655.35 and low <= distance < high):
        return None
    return round(low * 100), round(high * 100), round(distance * 100)


def planar_ned(odom):
    if odom.header.frame_id != 'lidar_odom' or odom.child_frame_id != 'base_link':
        return None
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    if not all(map(math.isfinite, (p.x, p.y, q.x, q.y, q.z, q.w))):
        return None
    if abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.0) > 0.01:
        return None
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    # The Gazebo plugin defines world +X as north and +Y as west.
    return p.x, -p.y, -yaw


class LidarFlightBridge(Node):

    def __init__(self):
        super().__init__('lidar_flight_bridge')
        self.target = pilot_altitude(self)
        self.link = None
        self.heartbeat = None
        self.last_heartbeat = self.next_connect = 0.0
        self.pose_stamp = self.range_stamp = self.height_stamp = None
        self.height = None
        self.pending = None
        self.ack_deadline = 0.0
        self.create_subscription(Odometry, '/nidarx_drone/lidar/odom',
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/nidarx_drone/downward_lidar/data',
                                 self.on_range, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/nidarx_drone/lidar/height',
                                 self.on_height, qos_profile_sensor_data)
        self.create_timer(0.05, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_service(Trigger, '/nidarx_drone/takeoff', self.takeoff,
                            callback_group=ReentrantCallbackGroup())

    def on_range(self, scan):
        stamp = stamp_ns(scan)
        bounds = range_cm(scan)
        now = self.get_clock().now().nanoseconds
        if (not self.heartbeat or bounds is None or not fresh(stamp, now)
                or (fresh(self.range_stamp, now) and stamp <= self.range_stamp)):
            return
        try:
            self.link.mav.distance_sensor_send(
                (stamp // 1_000_000) % (1 << 32), *bounds, MAV.MAV_DISTANCE_SENSOR_LASER,
                0, MAV.MAV_SENSOR_ROTATION_PITCH_270, 255)
        except OSError:
            self.close_link()
            return
        self.range_stamp = stamp

    def on_odom(self, odom):
        stamp = stamp_ns(odom)
        pose = planar_ned(odom)
        now = self.get_clock().now().nanoseconds
        if (not self.heartbeat or pose is None or not fresh(stamp, now)
                or (fresh(self.pose_stamp, now) and stamp <= self.pose_stamp)):
            return
        north, east, yaw = pose
        # The EKF selects only XY and yaw from this planar source.
        try:
            self.link.mav.vision_position_estimate_send(
                stamp // 1000, north, east, 0.0, 0.0, 0.0, yaw)
        except OSError:
            self.close_link()
            return
        self.pose_stamp = stamp

    def on_height(self, msg):
        stamp = stamp_ns(msg)
        now = self.get_clock().now().nanoseconds
        if (msg.header.frame_id == 'lidar_odom' and math.isfinite(msg.point.z)
                and fresh(stamp, now)
                and (not fresh(self.height_stamp, now) or stamp > self.height_stamp)):
            self.height = msg.point.z
            self.height_stamp = stamp

    def close_link(self):
        if self.pending and not self.pending.done():
            self.pending.set_result((False, 'Lost the ArduCopter takeoff link'))
        if self.link:
            self.link.close()
        self.link = self.heartbeat = None
        self.pose_stamp = self.range_stamp = self.height_stamp = None

    def destroy_node(self):
        self.close_link()
        return super().destroy_node()

    def poll(self):
        now = time.monotonic()
        if self.pending and not self.pending.done() and now >= self.ack_deadline:
            self.pending.set_result((False, 'ArduCopter takeoff acknowledgement timed out'))
        if self.link is None:
            if now < self.next_connect:
                return
            self.next_connect = now + 1.0
            try:
                self.link = mavutil.mavlink_connection(
                    'tcp:127.0.0.1:5762', source_system=1,
                    source_component=MAV.MAV_COMP_ID_ONBOARD_COMPUTER, retries=1)
            except OSError:
                return
            self.last_heartbeat = now
        try:
            for _ in range(100):
                msg = self.link.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_srcComponent() != MAV.MAV_COMP_ID_AUTOPILOT1:
                    continue
                if (msg.get_type() == 'HEARTBEAT'
                        and msg.autopilot == MAV.MAV_AUTOPILOT_ARDUPILOTMEGA):
                    if self.heartbeat is None:
                        self.get_logger().info('Connected to ArduCopter SITL')
                    self.heartbeat = msg
                    self.last_heartbeat = now
                elif (msg.get_type() == 'COMMAND_ACK' and self.heartbeat
                      and msg.get_srcSystem() == self.heartbeat.get_srcSystem()
                      and msg.command == MAV.MAV_CMD_NAV_TAKEOFF and self.pending
                      and not self.pending.done()):
                    accepted = msg.result == MAV.MAV_RESULT_ACCEPTED
                    result = MAV.enums['MAV_RESULT'].get(msg.result)
                    reason = result.name if result else str(msg.result)
                    self.pending.set_result((accepted, f'ArduCopter takeoff: {reason}'))
        except OSError:
            self.close_link()
            return
        if now - self.last_heartbeat > 4.0:
            self.close_link()

    async def takeoff(self, request, response):
        now = self.get_clock().now().nanoseconds
        samples = (self.pose_stamp, self.range_stamp, self.height_stamp)
        if self.pending is not None:
            response.message = 'A takeoff request is pending'
        elif not self.heartbeat or time.monotonic() - self.last_heartbeat > 2.5:
            response.message = 'Waiting for an ArduCopter SITL heartbeat'
        elif (self.heartbeat.custom_mode != MAV.COPTER_MODE_GUIDED
              or not self.heartbeat.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED):
            response.message = 'Select GUIDED and arm in the pilot console first'
        elif not all(fresh(stamp, now) for stamp in samples):
            response.message = 'Waiting for fresh planar odometry, downward range and height'
        elif self.height >= self.target:
            response.message = 'Current height is already at or above pilot_altitude'
        else:
            # WP_RFND_USE=0 selects the above-home Guided takeoff convention.
            # Home is set at arming, with base_link above the floor on the legs.
            self.pending = Future(executor=self.executor)
            self.ack_deadline = time.monotonic() + 3.0
            try:
                self.link.mav.command_int_send(
                    self.heartbeat.get_srcSystem(), MAV.MAV_COMP_ID_AUTOPILOT1,
                    MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, 0, self.target - self.height)
                response.success, response.message = await self.pending
            except OSError as error:
                response.message = f'Takeoff link error: {error}'
            finally:
                self.pending = None
        return response


def main(args=None):
    run_node(LidarFlightBridge, args)


if __name__ == '__main__':
    main()
