#!/usr/bin/env python

"""
Description:
    Publish a LaserScan message with random ranges between 1 and 10.
"""
import random

import rospy
from sensor_msgs.msg import LaserScan

def talker():
    pub = rospy.Publisher("fake_scan", LaserScan, queue_size=50) # specify the published topic name
    rospy.init_node("fake_scan_publihser") # initialize the node with the node name
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = -2.094395
        scan.angle_max = 2.094395
        scan.angle_increment = 0.010472
        scan.range_min = 1.0
        scan.range_max = 10.0
        length = 401
        scan.ranges = []
        for i in range(length):
            scan.ranges.append(random.uniform(scan.range_min, scan.range_max))
        
        pub.publish(scan)
        rate.sleep()

  
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

