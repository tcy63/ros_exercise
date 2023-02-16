#!/usr/bin/env python

import rosbag
import rospy
import tf
import tf2_ros
import numpy as np

from tf.transformations import quaternion_matrix, quaternion_from_matrix

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

def dynamic_publisher():
    rospy.init_node("dynamic_tf_cam_publisher.py")
    
    # Create a listener first
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        # Fetch the transform
        transform = tfBuffer.lookup_transform("world", "base_link_gt", rospy.Time())   # target frame, source fram, time

        base_translation = transform.translation
        base_rotation = transform.rotation # quaternion
        base_rotation_matrix = quaternion_matrix(base_rotation)
        
        # Construct the transform matrix from world to base_link_gt
        base_world_matrix = np.vstack((np.hstack((base_rotation_matrix, base_translation.reshape(-1, 1))), [0, 0, 0, 1]))

        # Construct the transform matrix from base_link_gt to cameras
        left_base_matrix = np.array(
                [
                    [1, 0, 0, -0.05],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ]
            )

        base_left_matrix = np.linalg.inv(left_base_matrix)

        right_base_matrix = np.array(
                [
                    [1, 0, 0, 0.05],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ]    
            )

        # Compose the tranform matrices
        left_world_matrix = np.matmul(left_base_matrix, base_world_matrix)

        right_left_matrix = np.matmul(right_base_matrix, base_left_matrix)
        
        # Broadcast the transforms to the TF tree
        broadcaster = tf2_ros.TransformBroadcaster()
        
        # Initialize a transform object
        left_transform = TransformStamped()
        
        # Add a timestamp
        left_transform.header.stamp = rospy.Time.now()

        # Add the source and target frame
        left_transform.header.frame_id = "world"
        left_transform.child_frame_id = "left_cam"

        # Add the translation
        left_transform.transform.translation = left_world_matrix[:, -1][:-1]

        # Add the rotation
        left_transform.transform.rotation = quaternion_from_matrix(left_world_matrix[:, :3][:3, :])

        # Send the transform
        broadcaster.sendTransform(left_transform)

        r.sleep()


if __name__ == '__main__':
    

