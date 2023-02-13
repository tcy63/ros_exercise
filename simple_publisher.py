#!/usr/bin/env python

import random
import rospy
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher("my_random_float", Float32, queue_size=10)
    rospy.init_node("simple_publisher") # initialize the ROS node
    rate = rospy.Rate(20)   # 20hz
    while not rospy.is_shutdown():
        random_number = random.uniform(0,10.0) 
        rospy.loginfo(random_number)
        pub.publish(random_number)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
