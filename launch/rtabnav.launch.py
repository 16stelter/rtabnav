import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    # Get directories
    package_dir = get_package_share_directory('rtabnav')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ',
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

    declare_remap_file_cmd = DeclareLaunchArgument(
        'remap_file',
        default_value=os.path.join(package_dir, 'params', 'leo_nav_remaps.yaml'),
        description='Full path to the ROS2 remap file to use for all launched nodes',
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_params_file_cmd,
        declare_remap_file_cmd,
        declare_rviz_config_file_cmd,
        OpaqueFunction(function=launch_nodes)
    ])

def launch_nodes(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    rviz_config_file = LaunchConfiguration('rviz_config_file').perform(context)
    remap_file = LaunchConfiguration('remap_file').perform(context)
    launch_dir = os.path.join(get_package_share_directory('rtabnav'), 'launch')


    namespaced_params_file = ReplaceString(
        source_file=params_file,
        replacements={
            '<robot_namespace>': PythonExpression([
                '"" if "', namespace, '" == "" else "/" + "', namespace + '"'
            ]),
        },
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=namespaced_params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = load_remappings(remap_file)

    # RViz configuration
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )

    start_rtabmap_cmd = Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      parameters=[configured_params, {'use_sim_time': use_sim_time.lower() == 'true'}],
      arguments=['--ros-args', '--log-level', 'info'],
      namespace=namespace,
      remappings=remappings,
    )

    # Bringup commands for the navigation stack
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'nav2.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': namespaced_params_file,
            'remap_file': remap_file,
        }.items(),
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    return [stdout_linebuf_envvar, start_rtabmap_cmd, rviz_cmd, start_nav2_cmd]

def load_remappings(remap_file): 
    with open(remap_file, 'r') as f: 
        data = yaml.safe_load(f) 
        return [(item['from'], item['to']) for item in data['remappings']]