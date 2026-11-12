import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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

    # ── RViz2 ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        rsp_node,
        gazebo_launch,
        spawn_entity,
        rviz_node,
    ])
