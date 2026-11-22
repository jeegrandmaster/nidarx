"""Shared ROS node startup and measurement conventions for indoor lidar flight."""

import math

import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException


def pilot_altitude(node):
    target = node.declare_parameter(
        'pilot_altitude', 1.5,
        ParameterDescriptor(read_only=True, description='Base height above the floor in metres'),
    ).value
    if not 0.5 <= target <= 3.5:
        raise ValueError('pilot_altitude must be between 0.5 and 3.5 metres')
    return target


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def quaternion_matrix(q):
    xyzw = np.array([q.x, q.y, q.z, q.w])
    length = np.linalg.norm(xyzw)
    if not math.isfinite(length) or length < 1e-6:
        return None
    x, y, z, w = xyzw / length
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def run_node(node_class, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can interrupt DDS message conversion when the context shuts down.
        if node is None or node.context.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
