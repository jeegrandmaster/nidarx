"""Check the sensor poses that Gazebo Classic actually loads after joint reduction."""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro


def test_simulated_lidar_beams_match_published_frames(tmp_path):
    description = Path(__file__).resolve().parents[1] / 'description/nidarx_drone.urdf.xacro'
    urdf = tmp_path / 'drone.urdf'
    urdf.write_text(xacro.process_file(str(description)).toxml())
    robot = ET.parse(urdf)
    # Gazebo 11 uses SDFormat 9. Its fixed-joint reduction can silently drop
    # poses specified on sensors, even when the URDF and TF look correct.
    sdf = ET.fromstring(subprocess.check_output(['ign', 'sdf', '--force-version', '9',
                                                 '-p', str(urdf)]))
    for name, expected_z, expected_axis in (
            ('downward_lidar', -0.045, (0., 0., -1.)),
            ('rplidar', 0.050, (1., 0., 0.))):
        sensor = sdf.find(f".//link[@name='base_link']/sensor[@name='{name}_sensor']")
        frame = sensor.findtext('plugin/frame_name')
        joint = next(j for j in robot.findall('joint') if j.find('child').get('link') == frame)
        assert joint.find('parent').get('link') == 'base_link'
        origin = joint.find('origin')
        tf_pose = list(map(float, (origin.get('xyz') + ' ' + origin.get('rpy')).split()))
        ray_pose = list(map(float, sensor.findtext('pose').split()))
        for x, y, z, roll, pitch, yaw in (tf_pose, ray_pose):
            assert (x, y, z) == pytest.approx((0., 0., expected_z))
            beam = (math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch),
                    -math.sin(pitch))
            assert beam == pytest.approx(expected_axis, abs=1e-5)
            if name == 'rplidar':
                assert math.cos(roll) * math.cos(pitch) == pytest.approx(1.)
