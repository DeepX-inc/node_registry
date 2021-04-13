#!/usr/bin/env python3

import os

import pytest

from test_common_utils.test_config_utils import TestConfigUtils
from test_common_utils.test_utils import TestUtils


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
