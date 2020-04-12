#!/usr/bin/env python3
# Copyright 2020 Gaitech Korea Co., Ltd.
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

# Author: Brighten Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  config_dir = LaunchConfiguration('config_dir', 
    default=os.path.join(get_package_share_directory('twist_mux'), 
    'config', 'twist_mux_topics.yaml'))

  return LaunchDescription([
    Node(
      package='twist_mux', 
      node_executable='twist_mux', 
      output='screen',
      remappings={('/cmd_vel_out', '/twist_mux/cmd_vel')},
      parameters=[config_dir]),

    Node(
      package='twist_mux', 
      node_executable='twist_marker', 
      output='screen',
      remappings={('/twist', '/twist_mux/cmd_vel')},
      parameters=[{
        'frame_id': 'base_link',
        'scale': 1.0,
        'vertical_position': 2.0}])
])