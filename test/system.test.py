# Copyright 2019 Canonical, Ltd
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
import unittest

from ament_index_python import get_package_share_directory

import launch
import launch.actions
from launch.actions.execute_process import ExecuteProcess

import launch_ros.actions

import launch_testing


def generate_test_description(ready_fn):
    # Necessary to get real-time stdout from python processes:
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    parameters_file = os.path.join(
        get_package_share_directory('twist_mux'),
        'test', 'system_config.yaml'
    )

    twist_mux = launch_ros.actions.Node(
            package='twist_mux', node_executable='twist_mux',
            parameters=[parameters_file], env=proc_env)

    publisher = ExecuteProcess(
        cmd=['ros2 topic pub /lock_1 std_msgs/Bool "data: False" -r 20'],
        shell=True, env=proc_env
    )

    system_blackbox = launch.actions.Node(
            package='twist_mux', node_executable='system_blackbox.py', env=proc_env)

    return launch.LaunchDescription([
        twist_mux,
        publisher,
        system_blackbox,
        # Start tests right away - no need to wait for anything
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
    ])


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self):
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(self.proc_info)
