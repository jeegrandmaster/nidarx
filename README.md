# NidarX indoor lidar mapping

In the Gazebo indoor world, ArduCopter holds horizontal position and heading using the 360° lidar and holds height using the downward rangefinder. Choose a height for the centre of the drone (`base_link`) above the flat floor. The horizontal scanner sits 5 cm above that point.

## Fly and map

1. From the workspace root, build and launch the ROS 2 Humble package. The simulation uses Gazebo Classic 11, the ArduPilot Gazebo plugin, ArduCopter SITL, MAVProxy and SLAM Toolbox. The indoor parameter file was tested with the project's ArduCopter 4.8.0-dev build. Use the `sitl_bin` and `mavproxy_bin` launch arguments if those executables are installed elsewhere.

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-select nidarx_drone
   source install/setup.bash
   ros2 launch nidarx_drone launch_sim.launch.py pilot_altitude:=1.5
   ```

   The default is 1.5 m. The supplied 4 m room accepts targets from 0.5 to 3.5 m. For a headless run, add `gui:=false start_mavproxy:=false` and connect your pilot console to `tcp:127.0.0.1:5760`.

2. In MAVProxy, wait for the EKF to acquire its lidar position estimate, then select Guided and arm. Wait for the armed confirmation.

   ```text
   mode GUIDED
   arm throttle
   ```

3. In a second terminal at the workspace root, request the climb.

   ```bash
   source install/setup.bash
   ros2 service call /nidarx_drone/takeoff std_srvs/srv/Trigger '{}'
   ```

   The service requires recent position, range and height measurements. A successful response means ArduCopter accepted the takeoff command. RViz shows the occupancy grid as **Flight plane map** after the measured height stays within 10 cm of the target and tilt stays within 3° for one second.

4. Leave the drone in Guided to hold position. In MAVProxy, `position 1 0 0` requests a one-metre move forward at the current height. During translation the mapper uses IMU tilt and measured height to keep only lidar returns within 10 cm of the selected scan plane. Leaving the height band, tilting past 8°, or losing height data pauses insertion until the drone settles again. Use `mode LAND` to land.

5. To save the observed map from a sourced terminal while the simulation is running, use the Nav2 map saver.

   ```bash
   ros2 run nav2_map_server map_saver_cli -f indoor_map --fmt png --free 0.19 --ros-args -p map_subscribe_transient_local:=true
   ```

The saved map from the 1.5 m test flight shows all four walls of the simulated room. Black cells are occupied, white cells are free and grey cells are unobserved. [View the occupancy map](maps/indoor_1p5m.png) or [load it with the Nav2 metadata](maps/indoor_1p5m.yaml).

## Sensor data and frames

| Topic | Measurement |
| --- | --- |
| `/nidarx_drone/lidar/odom` | Scan-matched horizontal position and yaw at the horizontal scanner's 5.5 Hz update rate. Weak matches and excessive tilt suppress the estimate. |
| `/nidarx_drone/lidar/height` | Tilt- and mount-corrected body height above the floor in `point.z`, in metres. |
| `/nidarx_drone/mapping_scan` | Horizontal scans paired with an accepted pose at the same timestamp and admitted at the selected height. |
| `/map` | SLAM Toolbox occupancy grid at 5 cm per cell. |
| `/nidarx_drone/odom` | Gazebo ground truth for measuring flight error. |

The companion link on local SITL port 5762 feeds horizontal pose and raw downward range to EKF3. The ROS transform chain is `map → lidar_odom → base_link`. The body transform combines scan-matched position and yaw, IMU roll and pitch, and lidar height.
