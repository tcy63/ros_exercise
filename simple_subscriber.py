#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('simple_subscriber')
    rospy.Subscriber("my_random_float", Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
