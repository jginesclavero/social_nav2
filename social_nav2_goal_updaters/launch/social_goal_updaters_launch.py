# Copyright 2019 Intelligent Robotics Lab
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, EmitEvent
from launch_ros.events.lifecycle import ChangeState
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
import launch.events
import lifecycle_msgs.msg

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('social_nav2_goal_updaters')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Specify the actions
    escort_cmd = LifecycleNode(
        package='social_nav2_goal_updaters',
        executable='escort_goal_updater_node',
        name='escort_goal_updater_node',
        output='screen',
        parameters=[])
        
    follow_cmd = LifecycleNode(
        package='social_nav2_goal_updaters',
        executable='follow_goal_updater_node',
        name='follow_goal_updater_node',
        output='screen',
        parameters=[])
    hri_cmd = LifecycleNode(
        package='social_nav2_goal_updaters',
        executable='hri_goal_updater_node',
        name='hri_goal_updater_node',
        output='screen',
        parameters=[])

    emit_event_to_request_that_escort_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(escort_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    emit_event_to_request_that_follow_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(follow_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    emit_event_to_request_that_hri_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(hri_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(escort_cmd)
    ld.add_action(follow_cmd)
    ld.add_action(hri_cmd)

    ld.add_action(emit_event_to_request_that_escort_configure_transition)
    ld.add_action(emit_event_to_request_that_follow_configure_transition)
    ld.add_action(emit_event_to_request_that_hri_configure_transition)

    return ld
