# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import sys
import numpy as np
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
import os

def try_cv2_import():
    """
    In order to import cv2 in python3 we need to remove
    the python2.7 path from sys.path. To use the Habitat-PyRobot integration the user
    needs to export environment variable ROS_PATH which will look something like:
    /opt/ros/kinetic/lib/python2.7/dist-packages
    """
    import sys
    import os

    ros_path_kinetic = "/opt/ros/kinetic/lib/python2.7/dist-packages"
    ros_path_melodic = "/opt/ros/melodic/lib/python2.7/dist-packages"

    if ros_path_kinetic in sys.path:
        sys.path.remove(ros_path_kinetic)
        import cv2

        sys.path.append(ros_path_kinetic)
    elif ros_path_melodic in sys.path:
        sys.path.remove(ros_path_melodic)
        import cv2

        sys.path.append(ros_path_melodic)
    else:
        import cv2

    return cv2

def append_namespace(ns, rostopic):
    return os.path.join("/", os.path.join(ns, rostopic.strip("/")))

def list_to_pose(pose_list):
    pose_msg = Pose()
    if len(pose_list) == 7:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]
    elif len(pose_list) == 6:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        q = tf.transformations.quaternion_from_euler(
            pose_list[3], pose_list[4], pose_list[5]
        )
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
    return pose_msg


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
        tf_listener.waitForTransform(
            tgt_frame, src_frame, rospy.Time(0), rospy.Duration(3)
        )
        (trans, quat) = tf_listener.lookupTransform(tgt_frame, src_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        raise RuntimeError(
            "Cannot fetch the transform from"
            " {0:s} to {1:s}".format(tgt_frame, src_frame)
        )
    return trans, quat


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
    return tf.transformations.quaternion_from_euler(
        euler[0], euler[1], euler[2], axes="rzyx"
    )


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


def pix_to_3dpt(depth_im, rs, cs, intrinsic_mat, depth_map_factor, reduce=None, k=5):
    """
    Get the 3D points of the pixels in RGB images.
    :param depth_im: depth image (shape: :math:`[H, W]`)
    :param rs: rows of interest in the RGB image.
               It can be a list or 1D numpy array
               which contains the row indices.
               The default value is None,
               which means all rows.
    :param cs: columns of interest in the RGB image.
               It can be a list or 1D numpy array
               which contains the column indices.
               The default value is None,
               which means all columns.
    :param intrinsic_mat: np.ndarray [3,3],  the camera intrinsic matrix
    :param depth_map_factor: float,  factor by which depth img intensity value to be divide to get real depth
    :param reduce: whether to consider the depth at nearby pixels
                'none': no neighbour consideration
                'mean': depth based on the mean of kernel sized k  centered at [rs,cs]
                'max': depth based on the max of kernel sized k  centered at [rs,cs]
                'min': depth based on the min of kernel sized k  centered at [rs,cs]
    :param k: kernel size for reduce type['mean', 'max', 'min']

    :type rs: list or np.ndarray
    :type cs: list or np.ndarray
    :type reduce: str
    :tyep k: int

    :returns: tuple (pts, colors)

              pts: point coordinates in world frame
              (shape: :math:`[N, 3]`)

              colors: rgb values for pts_in_cam
              (shape: :math:`[N, 3]`)

    :rtype: np.ndarray
    """
    assert isinstance(rs, int) or isinstance(rs, list) or isinstance(rs, np.ndarray)
    assert isinstance(cs, int) or isinstance(cs, list) or isinstance(cs, np.ndarray)
    if isinstance(rs, int):
        rs = [rs]
    if isinstance(cs, int):
        cs = [cs]
    if isinstance(rs, np.ndarray):
        rs = rs.flatten()
    if isinstance(cs, np.ndarray):
        cs = cs.flatten()
    R, C = depth_im.shape
    if reduce == "none" or reduce is None:
        depth_im = depth_im[rs, cs]
    elif reduce == "mean":
        depth_im = np.array(
            [
                np.mean(
                    depth_im[
                        max(i - k, 0) : min(i + k, R), max(j - k, 0) : min(j + k, C)
                    ]
                )
                for i, j in zip(rs, cs)
            ]
        )
    elif reduce == "max":
        depth_im = np.array(
            [
                np.max(
                    depth_im[
                        max(i - k, 0) : min(i + k, R), max(j - k, 0) : min(j + k, C)
                    ]
                )
                for i, j in zip(rs, cs)
            ]
        )
    elif reduce == "min":
        depth_im = np.array(
            [
                np.min(
                    depth_im[
                        max(i - k, 0) : min(i + k, R), max(j - k, 0) : min(j + k, C)
                    ]
                )
                for i, j in zip(rs, cs)
            ]
        )
    else:
        raise ValueError(
            "Invalid reduce name provided, only the following"
            " are currently available: [{}, {}, {}, {}]".format(
                "none", "mean", "max", "min"
            )
        )

    depth = depth_im.reshape(-1) / depth_map_factor
    img_pixs = np.stack((rs, cs)).reshape(2, -1)
    img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
    uv_one = np.concatenate((img_pixs, np.ones((1, img_pixs.shape[1]))))

    intrinsic_mat_inv = np.linalg.inv(intrinsic_mat)
    uv_one_in_cam = np.dot(intrinsic_mat_inv, uv_one)
    pts_in_cam = np.multiply(uv_one_in_cam, depth)
    pts_in_cam = np.concatenate((pts_in_cam, np.ones((1, pts_in_cam.shape[1]))), axis=0)
    return pts_in_cam
