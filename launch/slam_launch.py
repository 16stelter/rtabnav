# Copyright (c) 2020 Samsung Research Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Getting directories and launch-files
    package_dir = get_package_share_directory('rtabnav')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )


    return LaunchDescription([
        declare_namespace_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_log_level_cmd,
        OpaqueFunction(function=launch_nodes),
    ])

def launch_nodes(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    log_level = LaunchConfiguration('log_level').perform(context)
    remap_file = LaunchConfiguration('remap_file').perform(context)

    lifecycle_nodes = ['map_saver']

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = load_remappings(remap_file)

    # Nodes launching commands
    start_map_server = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                name='map_saver',
                namespace=namespace,
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace=namespace,
                name='lifecycle_manager_slam',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
                remappings=remappings,
            ),
        ]
    )

    start_rtabmap_cmd = Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      parameters=[configured_params, {'use_sim_time': use_sim_time}],
      arguments=['--ros-args', '--log-level', log_level],
      namespace=namespace,
      remappings=remappings,
    )

    return [start_map_server, start_rtabmap_cmd]


def load_remappings(remap_file): 
    with open(remap_file, 'r') as f: 
        data = yaml.safe_load(f) 
        return [(item['from'], item['to']) for item in data['remappings']]