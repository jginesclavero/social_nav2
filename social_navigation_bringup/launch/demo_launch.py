# Copyright (c) 2018 Intel Corporation
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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, 
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    social_bringup_dir = get_package_share_directory('social_navigation_bringup')
    pedsim_dir = get_package_share_directory('pedsim_simulator')

    # nav_bringup_dir = get_package_share_directory('nav2_bringup')

    launch_dir = os.path.join(social_bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    frame_id = LaunchConfiguration('frame_id')
    scene_file = LaunchConfiguration('scene_file')
    simulation_factor = LaunchConfiguration('simulation_factor')

    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='map', description='Reference frame')

    declare_scene_file_cmd = DeclareLaunchArgument(
        'scene_file', 
        default_value=os.path.join(pedsim_dir, 'scenarios', 'tb3_house_demo_crowd.xml'),
        description='')

    declare_simulation_factor_cmd = DeclareLaunchArgument(
        'simulation_factor', default_value='0.07',
        description='Simulator factor. 0.0 to get static agents')
    # Specify the actions

    social_nav_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'tb3_house_simulation_launch.py')))

    pedsim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pedsim_simulator'), 
            'launch',
            'simulator_launch.py')),
        launch_arguments={'scene_file': scene_file,
                          'simulation_factor': simulation_factor}.items())

    pedsim_visualizer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pedsim_visualizer'), 'launch', 'visualizer_launch.py')),
        launch_arguments={'frame_id': frame_id}.items()
    )

    dummy_set_agent_action_cmd = Node(
        package='social_navigation_tooling',
        node_executable='dummy_set_agent_action',
        node_name='dummy_set_agent_action',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_scene_file_cmd)
    ld.add_action(declare_simulation_factor_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(dummy_set_agent_action_cmd)
    # Add any conditioned actions
    ld.add_action(pedsim_cmd)
    ld.add_action(pedsim_visualizer_cmd)
    ld.add_action(social_nav_bringup_cmd)

    return ld
