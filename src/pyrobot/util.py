# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import PyKDL as kdl
import numpy as np
import rospy
import tf
import tf_conversions
from geometry_msgs.msg import PoseStamped


def get_tf_transform(tf_listener, tgt_frame, src_frame):
    """
    Uses ROS TF to lookup the current transform from tgt_frame to src_frame,
    If the returned transform is applied to data, it will transform data in
    the src_frame into the tgt_frame

    :param tgt_frame: target frame
    :param src_frame: source frame
    :type tgt_frame: string
    :type src_frame: string

    :returns: trans, translation (x,y,z)
    :rtype: tuple (of floats)
    :returns: quat, rotation as a quaternion (x,y,z,w)
    :rtype: tuple (of floats)
    """
    try:
        tf_listener.waitForTransform(tgt_frame, src_frame,
                                     rospy.Time(0),
                                     rospy.Duration(3))
        (trans, quat) = tf_listener.lookupTransform(tgt_frame,
                                                    src_frame,
                                                    rospy.Time(0))
    except (tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException):
        raise RuntimeError('Cannot fetch the transform from'
                           ' {0:s} to {1:s}'.format(tgt_frame, src_frame))
    return trans, quat


def transform_to_ros_pose(trans, quat, frame):
    """
    Converts transformation (translation, quaternion) into ROS PoseStamped

    :param trans: translation (x,y,z)
    :param quat: rotation as a quaternion (x,y,z,w)
    :param frame: reference frame of pose
    :type trans: tuple (of floats)
    :type quat: tuple (of floats)
    :type frame: string

    :returns: pose
    :rtype: ROS geometry_msgs/PoseStamped
    """
    pose = PoseStamped()
    pose.pose = tf_conversions.Pose(trans, quat)
    pose.header.frame_id = frame
    return pose


def quat_to_rot_mat(quat):
    """
    Convert the quaternion into rotation matrix. The quaternion we used
    here is in the form of [x, y, z, w]

    :param quat: quaternion [x, y, z, w] (shape: :math:`[4,]`)
    :type quat: numpy.ndarray

    :return: the rotation matrix (shape: :math:`[3, 3]`)
    :rtype: numpy.ndarray
    """
    return tf.transformations.quaternion_matrix(quat)[:3, :3]


def euler_to_quat(euler):
    """
    Convert the yaw, pitch, roll into quaternion.

    :param euler: the yaw, pitch, roll angles (shape: :math:`[3,]`)
    :type quat: numpy.ndarray

    :return: quaternion [x, y, z, w] (shape: :math:`[4,]`)
    :rtype: numpy.ndarray
    """
    return tf.transformations.quaternion_from_euler(euler[0], euler[1],
                                                    euler[2], axes='rzyx')


def rot_mat_to_quat(rot):
    """
    Convert the rotation matrix into quaternion.

    :param quat: the rotation matrix (shape: :math:`[3, 3]`)
    :type quat: numpy.ndarray

    :return: quaternion [x, y, z, w] (shape: :math:`[4,]`)
    :rtype: numpy.ndarray
    """
    R = np.eye(4)
    R[:3, :3] = rot
    return tf.transformations.quaternion_from_matrix(R)


def kdl_array_to_numpy(kdl_data):
    np_array = np.zeros((kdl_data.rows(), kdl_data.columns()))
    for i in range(kdl_data.rows()):
        for j in range(kdl_data.columns()):
            np_array[i, j] = kdl_data[i, j]
    return np_array


def joints_to_kdl(joint_values):
    """
    Convert the numpy array into KDL data format

    :param joint_values: values for the joints
    :return: kdl data type
    """
    num_jts = joint_values.size
    kdl_array = kdl.JntArray(num_jts)
    for idx in range(num_jts):
        kdl_array[idx] = joint_values[idx]
    return kdl_array


def kdl_frame_to_numpy(frame):
    """
    Convert KDL Frame data into numpy array
    :param frame: data of KDL Frame
    :return: transformation matrix in numpy [4x4]
    """
    p = frame.p
    M = frame.M
    return np.array([[M[0, 0], M[0, 1], M[0, 2], p.x()],
                     [M[1, 0], M[1, 1], M[1, 2], p.y()],
                     [M[2, 0], M[2, 1], M[2, 2], p.z()],
                     [0, 0, 0, 1]])
