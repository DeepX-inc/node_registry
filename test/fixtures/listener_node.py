#!/usr/bin/env python3

from node_registry.decorators import register, rosnode
from std_msgs.msg import String


sub_topic = 'chatter'


@rosnode
def node():
    return 'listener_node'


@rosnode.subscribe(String, sub_topic, 1)
def callback(msg):
    rosnode.logger.info(f'I heard: [{msg.data}]')


register()
