#!/user/bin/env python

"""
Description: 
    Subscribes to the fake_scan topic published on by the fake_scan_publisher from 
    the previous exercises, and finds the longest return (the range element with the greatest value) 
    and publishes the corresponding angle and return value or distance.
File name: 
    open_space_publisher.py
Node Name: 
    open_space_publisher
Published topic names: 
    open_space/distance and open_space/angle
Message type: 
    Float32
Subscription topic names: 
    fake_scan
Publish rate: 
    20hz
"""
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

pub_dis = rospy.Publisher("open_space/distance", Float32)
pub_ang = rospy.Publisher("open_space/angle", Float32)

def callback(data):
    scan  = data

    angle_min = scan.angle_min
    angle_incremental = scan.angle_increment
    ranges = scan.ranges
    
    max_range = max(ranges)
    max_idx = ranges.index(max_range)
    angle = angle_min + angle_incremental * max_idx

    pub_ang.publish(angle)
    pub_dis.publish(max_range)

def listener():
    rospy.init_node("open_space_publisher")
    rospy.Subscriber("fake_scan", LaserScan, callback)
    rospy.spin()