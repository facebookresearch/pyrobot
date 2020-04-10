# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Interface for visual slam (ORB-SLAM2)
"""

import os
import threading
from os.path import expanduser

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from orb_slam2_ros.msg import Traj

from orb_slam2_ros.pcdlib import PointCloudProcessor


class VisualSLAM(object):
    """
    This class is used for fetching camera pose and base pose,
    and reconstructing the 3D world that the robot has seen so far.
    The engine behind this class is ORB-SLAM2.

    The origin of the fixed world coordinate frame is the base_link
    position in the first frame (when the program is just turned on).
    """

    def __init__(
        self,
        map_img_dir=os.path.join(expanduser("~"), ".ros/Imgs"),
        cam_pose_tp="/orb_slam2_rgbd/slam/camera_pose",
        cam_traj_tp="/orb_slam2_rgbd/slam/camera_traj",
        base_frame="/base_link",
        camera_frame="/camera_color_optical_frame",
        map_resolution=0.02,
        z_min=0.1,
        z_max=0.8,
        obstacle_cost=100,
        occ_map_rate=0.0,
        x_min=-5,
        y_min=-5,
        x_max=5,
        y_max=5,
    ):
        """
        The constructor for :class:`VisualSLAM` class.

        :param map_img_dir: parent directory of the saved
               RGB images and depth images

        :type map_img_dir: string

        """
        self.map_img_dir = map_img_dir
        self.cam_pose_tp = cam_pose_tp
        self.cam_traj_tp = cam_traj_tp
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.map_resultion = map_resolution
        self.z_min = z_min
        self.z_max = z_max
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.obstacle_cost = obstacle_cost
        rgb_dir = os.path.join(self.map_img_dir, "RGBImgs")
        depth_dir = os.path.join(self.map_img_dir, "DepthImgs")
        pcd_args = {
            "rgb_dir": rgb_dir,
            "depth_dir": depth_dir,
            "cfg_filename": "realsense_d435.yaml",
            "subsample_pixs": 1,
            "depth_threshold": (0.2, 1.5),
            "camera_traj_topic": self.cam_traj_tp,
        }
        self._pcd_processor = PointCloudProcessor(**pcd_args)
        self._cam_pose_in_cam = None
        self._cam_traj_in_cam = []
        self._cam_traj_lock = threading.Lock()
        self._cam_pose_sub = rospy.Subscriber(
            self.cam_pose_tp, PoseStamped, self._cam_pose_callback
        )
        self._cam_traj_sub = rospy.Subscriber(
            self.cam_traj_tp, Traj, self.cam_traj_callback
        )
        self.occ_map_pub = rospy.Publisher(
            "/occupancy_map", OccupancyGrid, queue_size=1
        )
        self.occ_map_msg = self._init_occupancy_map()
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)

        trans, rot, T = self.get_link_transform(self.camera_frame, self.base_frame)
        self._ini_base2cam_T = T
        self._ini_base2cam_trans = np.array(trans).reshape(-1, 1)
        self._ini_base2cam_rot = np.array(rot)
        if occ_map_rate > 0:
            rospy.Timer(
                rospy.Duration(int(np.ceil(1.0 / occ_map_rate))), self._callback_occ_pub
            )

    @property
    def camera_pose(self):
        """
        Returns the camera pose in the world frame

        :returns: (trans, rot, cam_pose_in_world)
                 trans: translational vector (shape: :math:`[3,]`)
                 rot: rotation matrix (shape: :math:`[3, 3]`)
                 cam_pose_in_world: homogeneous camera pose (shape: :math:`[4, 4]`)
        :rtype: (numpy.ndarray, numpy.ndarray, numpy.ndarray)
        """
        if self._cam_pose_in_cam is None:
            return None, None, None
        pos, ori = self._parse_pose_msg(self._cam_pose_in_cam)
        T = np.eye(4)
        T[:3, :3] = ori
        T[:3, 3] = pos
        cam_pose_in_world = self.cam_to_world(T)
        trans = cam_pose_in_world[:3, 3]
        rot = cam_pose_in_world[:3, :3]
        return trans, rot, cam_pose_in_world

    @property
    def camera_traj(self):
        """
        Returns the camera trajectory in the world frame

        :return: the camera trajectory in homogeneous form (shape: :math:`[N, 4, 4]`)
        :rtype: numpy.ndarray
        """
        cam_traj = []
        self._cam_traj_lock.acquire()
        for i in range(len(self._cam_traj_in_cam)):
            pos, ori = self._parse_pose_msg(self._cam_traj_in_cam[i])
            T = np.eye(4)
            T[:3, :3] = ori
            T[:3, 3] = pos
            cam_pose_in_world = self.cam_to_world(T)
            cam_traj.append(cam_pose_in_world.copy())
        self._cam_traj_lock.release()
        return np.asarray(cam_traj)

    @property
    def base_pose(self):
        """
        Returns the base pose in the world frame

        :returns: (trans, rot, base_pose)
                 trans: translational vector (shape: :math:`[3,]`)
                 rot: rotation matrix (shape: :math:`[3, 3]`)
                 base_pose: homogeneous base pose (shape: :math:`[4, 4]`)
        :rtype: (numpy.ndarray, numpy.ndarray, numpy.ndarray)
        """
        # gives the base pose as a tuple (x, y, yaw)
        trans, rot, T = self.get_link_transform(self.base_frame, self.camera_frame)
        _, _, camera_pose = self.camera_pose
        if camera_pose is None:
            return None, None, None
        base_pose = np.dot(camera_pose, T)
        rot = base_pose[:3, :3]
        trans = base_pose[:3, 3]
        return trans, rot, base_pose

    @property
    def base_pose_xyyaw(self):
        """
        Returns the (x, y, yaw) of the base. Note that here we assume
         that the robot moves on a flat floor

        :returns: (x, y, yaw)
        :rtype: (float, float, float)
        """
        trans, rot, T = self.base_pose
        if T is None:
            return None, None, None
        angle = np.arctan2(rot[1, 0], rot[0, 0])
        # angle, axis, _ = tf.transformations.rotation_from_matrix(T)
        x = trans[0]
        y = trans[1]
        yaw = angle
        return x, y, yaw

    def cam_to_world(self, pose_in_cam):
        """
        Convert the pose in the first camera frame to fixed world frame

        :param pose_in_cam: pose in the first camera frame,
                            (shape: :math:`[4, None]`)
        :type pose_in_cam: numpy.ndarray
        :return: pose in the world frame (shape: :math:`[4, None]`)
        :rtype: numpy.ndarray
        """
        pose_in_world = np.dot(self._ini_base2cam_T, pose_in_cam)
        return pose_in_world

    def get_3d_map(self):
        """
        Fetch the 3D point cloud that the robot has seen so far

        :return: (pts, colors)
                 pts: points of the point cloud (shape: :math:`[N, 3]`)
                 colors: color of the points (shape: :math:`[N, 3]`)
        :rtype: (numpy.ndarray, numpy.ndarray)
        """
        points, colors = self._pcd_processor.get_current_pcd()
        if points is None:
            return None, None

        pts = np.dot(points, self._ini_base2cam_rot.T)
        pts = pts + self._ini_base2cam_trans.T

        # points = np.concatenate((points, np.ones((points.shape[0], 1))), axis=1)
        # pts_in_world = np.dot(points, self._ini_base2cam_T.T)
        # pts = pts_in_world[:, :3]

        # points = points.T
        # homo_pts = np.concatenate((points, np.ones((1, points.shape[1]))),
        #                           axis=0)
        # pts_in_world = self.cam_to_world(homo_pts)
        # pts = pts_in_world[:3, :].T
        return pts, colors

    def _calc_grid_bds(self, points):
        if points.size == 0 or points is None:
            xMin = self.x_min
            xMax = self.x_max
            yMin = self.y_min
            yMax = self.y_max
            return
        mins = np.amin(points, axis=0)
        maxs = np.amax(points, axis=0)

        xMax = max(maxs[0], self.x_max)
        yMin = min(mins[1], self.y_min)
        xMin = min(mins[0], self.x_min)
        yMax = max(maxs[1], self.y_max)

        return xMin, yMin, xMax, yMax

    def get_occupancy_map(self):
        pts, colors = self.get_3d_map()
        if pts is None or pts.shape[0] < 1 or pts.size == 0:
            xcells = (int((self.x_max - self.x_min) / self.map_resultion)) + 1
            ycells = (int((self.y_max - self.y_min) / self.map_resultion)) + 1
            occ_map = np.zeros(xcells * ycells)
            return occ_map, xcells, ycells, self.x_min, self.y_min

        (self.x_min, self.y_min, self.x_max, self.y_max) = self._calc_grid_bds(pts)

        xcells = int(np.ceil((self.x_max - self.x_min) / self.map_resultion))
        ycells = int(np.ceil((self.y_max - self.y_min) / self.map_resultion))

        filter_idx = np.logical_and(pts[:, 2] >= self.z_min, pts[:, 2] <= self.z_max)

        pts = pts[filter_idx, :2]
        map_mins = np.array([self.x_min, self.y_min])
        pts = np.floor((pts - map_mins) / self.map_resultion).astype(int)
        pts = pts[:, 1] * xcells + pts[:, 0]
        occ_map = np.zeros(xcells * ycells)
        occ_map[pts] = self.obstacle_cost
        return occ_map, xcells, ycells, self.x_min, self.y_min

    def _pub_occupancy_map(self):
        occ_map, width, height, x_min, y_min = self.get_occupancy_map()
        if occ_map is None:
            return
        self.occ_map_msg.header.seq = self.occ_map_msg.header.seq + 1
        self.occ_map_msg.header.stamp = rospy.Time.now()
        self.occ_map_msg.info.map_load_time = rospy.Time.now()
        self.occ_map_msg.info.resolution = self.map_resultion
        self.occ_map_msg.info.width = width
        self.occ_map_msg.info.height = height
        self.occ_map_msg.info.origin.position.x = x_min
        self.occ_map_msg.info.origin.position.y = y_min
        self.occ_map_msg.data = occ_map
        self.occ_map_pub.publish(self.occ_map_msg)

    def _callback_occ_pub(self, event):
        self._pub_occupancy_map()

    def _parse_pose_msg(self, pose):
        """
        Convert the ros topic pose (geometry_msgs/Pose)
        into translational vector
        and the rotational vector

        :param pose: pose
        :type pose: geometry_msgs.Pose

        :return: (pos, ori)
                  pos: translational vector (shape: :math:`[3, ]`)
                  ori: rotational vector (shape: :math:`[3, 3]`)
        :rtype: (numpy.ndarray, numpy.ndarray)
        """
        pos = np.array(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        )
        quat = np.array(
            [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]
        )
        ori = tf.transformations.quaternion_matrix(quat)[:3, :3]
        return pos, ori

    def _cam_pose_callback(self, pose):
        """
        Store the pose from the pose subscriber

        :param pose: pose
        :type pose: geometry_msgs.Pose
        """
        self._cam_pose_in_cam = pose

    def cam_traj_callback(self, traj_data):
        """
        Store the trajectory from the trajectory subscriber

        :param traj_data: trajectory data
        :type traj_data: orb_slam2_ros.msg.Traj
        """
        self._cam_traj_lock.acquire()
        self._cam_traj_in_cam = traj_data.poses
        self._cam_traj_lock.release()

    def get_link_transform(self, src, tgt):
        """
        Returns the latest transformation from the target_frame to the source frame,
        i.e., the transform of source frame w.r.t target frame. If the returned
        transform is applied to data, it will transform data in the source_frame into
         the target_frame

        For more information, please refer to
        http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms

        :param src: source frame
        :param tgt: target frame
        :type src: string
        :type tgt: string

        :returns: (trans, rot, T)
                  trans: translational vector (shape: :math:`[3,]`)
                  rot: rotation matrix (shape: :math:`[3, 3]`)
                  T: transofrmation matrix (shape: :math:`[4, 4]`)
        :rtype: (numpy.ndarray, numpy.ndarray, numpy.ndarray)
        """
        trans, quat = self._get_tf_transform(self.tf_listener, tgt, src)
        rot = tf.transformations.quaternion_matrix(quat)[:3, :3]
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = trans
        return trans, rot, T

    def _init_occupancy_map(self):
        grid = OccupancyGrid()
        grid.header.seq = 1
        grid.header.frame_id = "/map"
        grid.info.origin.position.z = 0
        grid.info.origin.orientation.x = 0
        grid.info.origin.orientation.y = 0
        grid.info.origin.orientation.z = 0
        grid.info.origin.orientation.w = 1
        return grid

    def _get_tf_transform(self, tf_listener, tgt_frame, src_frame):
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
            (trans, quat) = tf_listener.lookupTransform(
                tgt_frame, src_frame, rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            raise RuntimeError(
                "Cannot fetch the transform from"
                " {0:s} to {1:s}".format(tgt_frame, src_frame)
            )
        return trans, quat


def main():
    rospy.init_node("vslam", anonymous=True)
    vslam = VisualSLAM()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
