#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_matrix

from geometry_msgs.msg import TransformStamped

import numpy as np

def main():
    rospy.init_node("base_link_tf_pub.py")
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        left_world_trans = tfBuffer.lookup_transform("world", "left_cam", rospy.Time(), rospy.Duration(2.0))

        left_world_quat = left_world_trans.transform.rotation
        left_world_rotation = [left_world_quat.x, left_world_quat.y, left_world_quat.z, left_world_quat.w]
        left_world_matrix = quaternion_matrix(left_world_rotation)
        world_left_matrix = np.linalg.inv(left_world_matrix)

        left_base_matrix = np.array(
            [
                [1, 0, 0, -0.05],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        )
        base_left_matrix = np.linalg.inv(left_base_matrix)

    
        # Compose transforms
        base_world_matrix = np.matmul(base_left_matrix, left_world_matrix)
        
        # Create a broadcaster for the transform from base_link_gt to world
        broadcaster = tf2_ros.TransformBroadcaster()

        # Create a transform object
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link_gt_2"

        base_world_translation = base_world_matrix[:, -1][:-1]
        transform.transform.translation.x = base_world_translation[0]
        transform.transform.translation.y = base_world_translation[1]
        transform.transform.translation.z = base_world_translation[2]

        base_world_quat = quaternion_from_matrix(base_world_matrix)
        transform.transform.rotation.x = base_world_quat[0]
        transform.transform.rotation.y = base_world_quat[1]
        transform.transform.rotation.z = base_world_quat[2]
        transform.transform.rotation.w = base_world_quat[3]

        broadcaster.sendTransform(transform)

if __name__ == '__main__':
    main()