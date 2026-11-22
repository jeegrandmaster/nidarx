import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('nidarx_drone')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'description', 'nidarx_drone.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'indoor.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    gazebo_share = '/usr/share/gazebo-11'
    gazebo_plugins = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'

    # Process xacro → URDF
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ── Declare launch arguments ──
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    pilot_altitude_arg = DeclareLaunchArgument(
        'pilot_altitude', default_value='1.5',
        description='Target base_link height above the indoor floor in metres (0.5 to 3.5)',
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', description='Show Gazebo and RViz')
    lidar_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'pilot_altitude': ParameterValue(LaunchConfiguration('pilot_altitude'), value_type=float),
    }
    sitl_bin_arg = DeclareLaunchArgument(
        'sitl_bin',
        default_value=os.environ.get(
            'ARDUPILOT_SITL_BIN',
            '/home/onlydrones/ardupilot/build/sitl/bin/arducopter',
        ),
        description='Path to the ArduCopter SITL executable',
    )
    sitl_params_arg = DeclareLaunchArgument(
        'sitl_params',
        default_value=os.path.join(pkg_share, 'config', 'nidarx_indoor.parm'),
        description='ArduCopter SITL parameter file',
    )
    start_sitl_arg = DeclareLaunchArgument(
        'start_sitl',
        default_value='true',
        description='Start ArduCopter SITL with Gazebo',
    )
    start_mavproxy_arg = DeclareLaunchArgument(
        'start_mavproxy',
        default_value='true',
        description='Start MAVProxy connected to the local SITL TCP link',
    )
    mavproxy_bin_arg = DeclareLaunchArgument(
        'mavproxy_bin',
        default_value=os.environ.get(
            'MAVPROXY_BIN',
            '/home/onlydrones/.local/bin/mavproxy.py',
        ),
        description='Path to the MAVProxy executable',
    )
    mavproxy_out_arg = DeclareLaunchArgument(
        'mavproxy_out',
        default_value='udp:127.0.0.1:14550',
        description='MAVProxy MAVLink output endpoint',
    )
    mavproxy_terminal_arg = DeclareLaunchArgument(
        'mavproxy_terminal',
        default_value='/usr/bin/xterm',
        description='Terminal emulator used to run the interactive MAVProxy console',
    )

    # ── Robot State Publisher ──
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    # ── Gazebo server + client ──
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
        }.items(),
    )

    # ── Spawn the drone into Gazebo ──
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'nidarx_drone',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.195',      # ground clearance = leg_length + offset
                ],
                output='screen',
            ),
        ],
    )

    lidar_odom = Node(
        package='nidarx_drone',
        executable='planar_lidar_odom.py',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    mapping_plane = Node(
        package='nidarx_drone', executable='mapping_plane.py',
        parameters=[lidar_params], output='screen',
    )
    flight_bridge = Node(
        package='nidarx_drone', executable='lidar_flight_bridge.py',
        parameters=[lidar_params], output='screen',
    )
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        parameters=[
            os.path.join(get_package_share_directory('slam_toolbox'), 'config',
                         'mapper_params_online_async.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'odom_frame': 'lidar_odom', 'base_frame': 'base_link',
             'scan_topic': '/nidarx_drone/mapping_scan',
             'resolution': 0.05, 'map_update_interval': 1.0,
             'min_laser_range': 0.16, 'max_laser_range': 12.0, 'min_pass_through': 0,
             # Different beams enter the slice as the drone levels, even at a fixed XY.
             'minimum_time_interval': 0.5, 'minimum_travel_distance': 0.0,
             'minimum_travel_heading': 0.0, 'use_scan_barycenter': False},
        ],
        output='screen',
    )

    sitl_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_sitl')),
        cmd=[
            LaunchConfiguration('sitl_bin'),
            '--wipe',
            '--model', 'gazebo-iris',
            '--speedup', '1',
            '--serial0', 'tcp:0',
            '--sim-address', '127.0.0.1',
            '--sim-port-in', '9003',
            '--sim-port-out', '9002',
            '--defaults', LaunchConfiguration('sitl_params'),
        ],
        output='screen',
    )

    mavproxy_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_mavproxy')),
        cmd=[
            LaunchConfiguration('mavproxy_terminal'),
            '-T', 'MAVProxy',
            '-hold',
            '-e',
            LaunchConfiguration('mavproxy_bin'),
            '--master=tcp:127.0.0.1:5760',
            '--out', LaunchConfiguration('mavproxy_out'),
            '--force-connected',
            '--console',
        ],
        output='screen',
    )
    mavproxy_process = TimerAction(
        period=5.0,
        actions=[mavproxy_process],
    )

    # ── RViz2 ──
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('gui')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value=os.pathsep.join(filter(None, [
                gazebo_share,
                os.environ.get('GAZEBO_RESOURCE_PATH', ''),
            ])),
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.pathsep.join(filter(None, [
                os.path.join(gazebo_share, 'models'),
                os.environ.get('GAZEBO_MODEL_PATH', ''),
            ])),
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=os.pathsep.join(filter(None, [
                gazebo_plugins,
                os.environ.get('GAZEBO_PLUGIN_PATH', ''),
            ])),
        ),
        SetEnvironmentVariable(
            name='GSETTINGS_BACKEND',
            value='memory',
        ),
        SetEnvironmentVariable(
            name='GSETTINGS_SCHEMA_DIR',
            value='/usr/share/glib-2.0/schemas',
        ),
        SetEnvironmentVariable(
            name='QT_QPA_PLATFORM',
            value='xcb',
        ),
        SetEnvironmentVariable(
            name='QT_STYLE_OVERRIDE',
            value='Fusion',
        ),
        use_sim_time_arg,
        pilot_altitude_arg,
        gui_arg,
        sitl_bin_arg,
        sitl_params_arg,
        start_sitl_arg,
        start_mavproxy_arg,
        mavproxy_bin_arg,
        mavproxy_out_arg,
        mavproxy_terminal_arg,
        rsp_node,
        gazebo_launch,
        sitl_process,
        mavproxy_process,
        spawn_entity,
        lidar_odom,
        mapping_plane,
        flight_bridge,
        slam,
        rviz_node,
    ])
