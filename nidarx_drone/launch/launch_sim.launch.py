import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


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
            'verbose': 'true',
        }.items(),
    )

    # ── Spawn the drone into Gazebo ──
    spawn_entity = Node(
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
    )

    sitl_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_sitl')),
        cmd=[
            LaunchConfiguration('sitl_bin'),
            '--model', 'gazebo-iris',
            '--speedup', '1',
            '--sim-address', '127.0.0.1',
            '--sim-port-in', '9003',
            '--sim-port-out', '9002',
            '--defaults', LaunchConfiguration('sitl_params'),
        ],
        output='screen',
    )

    # ── RViz2 ──
    rviz_node = Node(
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
        sitl_bin_arg,
        sitl_params_arg,
        start_sitl_arg,
        rsp_node,
        gazebo_launch,
        sitl_process,
        spawn_entity,
        rviz_node,
    ])
