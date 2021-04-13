#!/usr/bin/env python3

from node_registry.decorators import register, rosnode
from std_msgs.msg import String


topic = 'chatter'
counter = 0


@rosnode.publisher(String, topic, 1)
@rosnode
def node():
    return 'talker_node'


@rosnode.timer(1.0)
def timer_cb():
    global counter

    pub = rosnode.get_publisher(topic)
    pub.publish(String(data=f'Hello World: {counter}'))
    counter += 1


register()
