# Copyright 2019 Canonical, Ltd.
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
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
import launch_ros.actions

def generate_launch_description():
    # Necessary to get real-time stdout from python processes:
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    publisher = ExecuteProcess(
        cmd=['ros2 topic pub /chatter std_msgs/String "data: Hello World" -r 0.5'],
        shell=True,
        output='screen',
        env=proc_env
    )

    parameters_file = os.path.join(
        get_package_share_directory('twist_mux'),
        'config', 'twist_mux_locks.yaml'
    )

    twist_mux = launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='parameter_blackboard',
            parameters=[parameters_file,],
            env=proc_env)

    return LaunchDescription([publisher, twist_mux])
