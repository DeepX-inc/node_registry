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

import os

import pytest

_node_name = 'rosnode'


@pytest.mark.rostest
def generate_test_description():
    config_utils = TestConfigUtils(
        node_name=_node_name, test_dir=os.path.dirname(__file__)
    )
    return (config_utils.get_launch_description(), locals())


class TestRosNode(TestUtils):

    def test_check_all_topics(self):
        self.check_all_topics(expected_output=['/chatter'])

    def test_topic_endpoint_info(self):
        self.topic_endpoint_info(
            topic_name='/chatter',
            expected_output=[
                'Type: std_msgs/msg/String',
                'Publisher count: 1',
                'Subscription count: 1',
            ],
        )

    def test_topic_type(self):
        self.topic_type(
            topic_name='/chatter', expected_output=['std_msgs/msg/String']
        )
