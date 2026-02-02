import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    GroupAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.descriptions import ParameterValue, ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.actions import Node, PushROSNamespace


def generate_launch_description():
    # Get directories
    package_dir = get_package_share_directory('rover_demo_rtabnav')
    launch_dir = os.path.join(package_dir, 'launch')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    # Launch variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    remappings=[
        ('/scan', 'scan'),
        ('/scan_cloud', 'pointcloud/points'),
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/map', 'map'),
        ('/odom', 'odom'),
        ('odom', 'true_pose'),
        ('/cmd_vel', 'cmd_vel'),
        ('/imu', 'imu/data'),
    ]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='leo', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'params', 'leo_nav_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(package_dir, 'params', 'nav_rviz.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    # Remapping parameters if a namespace is used
    params_file = LaunchConfiguration('params_file')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # RViz configuration
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )

    # Bringup commands for the navigation stack
    bringup_cmd_group = GroupAction(
        [
            Node(
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                namespace=namespace,
                parameters=[configured_params, {'autostart': True}],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=remappings,
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'slam_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                   os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Launch description
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
