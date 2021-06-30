#!/usr/bin/env python3
# Copyright (c) 2021, DeepX-inc
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# @author Krishneel Chaudhary


import contextlib
import os
import sys
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

__PYTHONUNBUFFERED__ = '1'
__REPEAT__ = 3


@pytest.mark.rostest
def generate_test_description():
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = __PYTHONUNBUFFERED__
    this_dir = os.path.join(os.path.dirname(__file__), 'fixtures')
    path_to_talker = os.path.join(this_dir, 'talker_node.py')
    path_to_listener = os.path.join(this_dir, 'listener_node.py')
    pub_node = Node(
        executable=sys.executable,
        arguments=[path_to_talker],
        additional_env=proc_env,
    )
    sub_node = Node(
        executable=sys.executable,
        arguments=[path_to_listener],
        additional_env=proc_env,
    )
    return (
        LaunchDescription([
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'stop'],
                    name='daemon-stop',
                    on_exit=[
                        ExecuteProcess(
                            cmd=['ros2', 'daemon', 'start'],
                            name='daemon-start',
                            on_exit=[
                                sub_node,
                                pub_node,
                                launch_testing.actions.ReadyToTest(),
                            ],
                            additional_env=proc_env,
                        )
                    ],
                ),
            ]),
        locals(),
    )


class TestRosNode(unittest.TestCase):
    timeout: int = 10

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_topic_command(self, arguments):
            topic_command_action = ExecuteProcess(
                    cmd=['ros2', 'topic', *arguments],
                    additional_env={'PYTHONUNBUFFERED': __PYTHONUNBUFFERED__},
                    name='ros2topic-cli',
                    output='screen',
                )
            with launch_testing.tools.launch_process(
                    launch_service,
                    topic_command_action,
                    proc_info,
                    proc_output,
                    ) as topic_command:
                yield topic_command
        cls.launch_topic_command = launch_topic_command

    @launch_testing.markers.retry_on_failure(times=__REPEAT__, delay=1)
    def test_check_all_topics(self, expected_output=['/chatter']):
        with self.launch_topic_command(arguments=['list']) as topic_command:
            assert topic_command.wait_for_shutdown(self.timeout)
            assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=expected_output,
                text=topic_command.output,
                strict=False,
            ), f'Expected: {expected_output}, Received: {topic_command.output}'

    @launch_testing.markers.retry_on_failure(times=__REPEAT__, delay=1)
    def test_topic_endpoint_info(self, topic_name='/chatter', expected_output=[
                'Type: std_msgs/msg/String',
                'Publisher count: 1',
                'Subscription count: 1',
            ]):
        with self.launch_topic_command(
            arguments=['info', topic_name]
        ) as topic_command:
            assert topic_command.wait_for_shutdown(self.timeout)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        msg = ""
        for line in expected_output:
            msg += line+'\n'
        assert launch_testing.tools.expect_output(
            expected_text=msg,
            text=topic_command.output,
            strict=True,
        ), f'Expected: {expected_output}, Received: {topic_command.output}'

    @launch_testing.markers.retry_on_failure(times=__REPEAT__, delay=1)
    def test_topic_type(self, topic_name='/chatter', expected_output=[
                'std_msgs/msg/String']):
        with self.launch_topic_command(
            arguments=['type', topic_name]
        ) as topic_command:
            assert topic_command.wait_for_shutdown(self.timeout)
            assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=expected_output,
                text=topic_command.output,
                strict=True,
            )
