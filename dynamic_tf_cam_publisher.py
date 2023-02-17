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
        trans = tfBuffer.lookup_transform("world", "base_link_gt", rospy.Time(), rospy.Duration(2.0))   # target frame, source fram, time
        
        transform = trans.transform
        base_vect = transform.translation    # a Vector3 object
        base_translation = [base_vect.x, base_vect.y, base_vect.z]
        base_quat = transform.rotation # a Quaternion object
        base_rotation = [base_quat.x, base_quat.y, base_quat.z, base_quat.w]
        base_rotation_matrix = quaternion_matrix(base_rotation)[:, :3][:3, :]
        
        # Construct the transform matrix from world to base_link_gt
        base_world_matrix = np.vstack((np.hstack((base_rotation_matrix, np.array(base_translation)[:, np.newaxis])), [0, 0, 0, 1]))

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
        right_left_transform = TransformStamped()
        
        # Add a timestamp
        left_transform.header.stamp = rospy.Time.now()
        right_left_transform.header.stamp = rospy.Time.now()

        # Add the source and target frame
        left_transform.header.frame_id = "world"    # parent & target frame
        left_transform.child_frame_id = "left_cam"  # child & source frame
        right_left_transform.header.frame_id = "left_cam"   # parent & target frame
        right_left_transform.child_frame_id = "right_cam"   # child & source frame

        # Add the translation
        left_world_translation = left_world_matrix[:, -1][:-1]
        left_transform.transform.translation.x = left_world_translation[0]
        left_transform.transform.translation.y = left_world_translation[1]
        left_transform.transform.translation.z = left_world_translation[2]

        right_left_translation = right_left_matrix[:, -1][:-1]
        right_left_transform.transform.translation.x = right_left_translation[0]
        right_left_transform.transform.translation.y = right_left_translation[1]
        right_left_transform.transform.translation.z = right_left_translation[2]

        # Add the rotation
        left_world_quat = quaternion_from_matrix(left_world_matrix)
        left_transform.transform.rotation.x = left_world_quat[0]
        left_transform.transform.rotation.y = left_world_quat[1]
        left_transform.transform.rotation.z = left_world_quat[2]
        left_transform.transform.rotation.w = left_world_quat[3]

        right_left_quat = quaternion_from_matrix(right_left_matrix)
        right_left_transform.transform.rotation.x = right_left_quat[0]
        right_left_transform.transform.rotation.y = right_left_quat[1]
        right_left_transform.transform.rotation.z = right_left_quat[2]
        right_left_transform.transform.rotation.w = right_left_quat[3]

        # Send the transform
        broadcaster.sendTransform(left_transform)
        broadcaster.sendTransform(right_left_transform)

        r.sleep()


if __name__ == '__main__':
    dynamic_publisher()

