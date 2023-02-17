#!/usr/bin/env python

import rospy
import tf
import tf2_ros

from tf.transformations import quaternion_from_matrix
from tf2_ros import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped

import numpy as np

def make_transforms(source_frame, target_frame, transform_matrix):

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = target_frame
    t.child_frame_id = source_frame

    trans = transform_matrix[:, -1][:-1]
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]

    quat = quaternion_from_matrix(transform_matrix)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    return t


def main():
    rospy.init_node("static_tf_cam_publisher.py")

    broadcaster = StaticTransformBroadcaster()

    left_base_matrix = np.array(
        [
            [1, 0, 0, -0.05],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
    )
    left_base_trans = make_transforms('left_cam', 'base_link_gt', left_base_matrix)

    right_base_matrix = np.array(
        [
            [1, 0, 0, 0.05],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]    
    )
    right_base_trans = make_transforms('right_cam', 'base_link_gt', right_base_matrix)

    broadcaster.sendTransform([left_base_trans, right_base_trans])
    
    rospy.spin()

if __name__ == '__main__':
    main()
