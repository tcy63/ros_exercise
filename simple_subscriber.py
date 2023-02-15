#!/usr/bin/env python
"""
Description:
    Subscribes to the topic published on by the simple_publisher and publishes the natural log of the received messages.
"""
import math

import rospy
from std_msgs.msg import Float32

pub = rospy.Publisher("random_float_log", Float32, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub.publish(math.log(data.data))

def listener():
    rospy.init_node('simple_subscriber')
    rospy.Subscriber("my_random_float", Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
