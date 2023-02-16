#!/usr/bin/env python

"""
Description:
    Publish a LaserScan message with random ranges between 1 and 10.
"""
import random
import math

import rospy
from sensor_msgs.msg import LaserScan

def talker(publish_topic, publish_rate, angle_min, angle_max, angle_increment, range_min, range_max):
    pub = rospy.Publisher(publish_topic, LaserScan, queue_size=50)    
    rospy.init_node("fake_scan_publihser") # initialize the node with the node name

    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.range_min = range_min
        scan.range_max = range_max

        length = int((angle_max - angle_min) // angle_increment) + 1

        scan.ranges = []
        for i in range(length):
            scan.ranges.append(random.uniform(scan.range_min, scan.range_max))
        
        pub.publish(scan)
        rate.sleep()

  
if __name__ == '__main__':
    
    publish_topic = rospy.get_param("Publish topic", "fake_scan")
    
    publish_rate = rospy.get_param("Publish rate", 20)

    angle_min = rospy.get_param("Angle_min", -2*math.pi/3)
    angle_max = rospy.get_param("Angle_max", 2*math.pi/3)
    angle_increment = rospy.get_param("Angle_increment", math.pi/300)
    range_min = rospy.get_param("Range_min", 1.0)
    range_max = rospy.get_param("Range_max", 10.0)

    try:
        talker(publish_topic, publish_rate, angle_min, angle_max, angle_increment, range_min, range_max)
    except rospy.ROSInterruptException:
        pass

