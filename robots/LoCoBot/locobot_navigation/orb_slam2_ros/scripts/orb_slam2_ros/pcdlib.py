# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import os
import rospkg
import threading
import time
from os.path import expanduser

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import numpy as np
import rospy
import tf.transformations
import yaml

USE_ORB_SLAM2 = True
USE_OPEN3D = True

try:
    from orb_slam2_ros.msg import Traj
except:
    USE_ORB_SLAM2 = False
    pass

try:
    import open3d
except ImportError:
    USE_OPEN3D = False
    pass


class DepthImgProcessor:
    """
    This class transforms the depth image and rgb image to point cloud
    """

    def __init__(
        self,
        subsample_pixs=1,
        depth_threshold=(0, 1.5),
        cfg_filename="realsense_d435.yaml",
    ):
        """
        The constructor for :class:`DepthImgProcessor` class.

        :param subsample_pixs: sample rows and columns for the images
        :param depth_threshold: minimum and maximum of valid depth values
        :param cfg_filename: configuration file name for ORB-SLAM2

        :type subsample_pixs: int
        :type depth_threshold: tuple
        :type cfg_filename: string
        """
        assert (type(depth_threshold) is tuple and 0 < len(depth_threshold) < 3) or (
            depth_threshold is None
        )
        self.subsample_pixs = subsample_pixs
        self.depth_threshold = depth_threshold
        self.cfg_data = self.read_cfg(cfg_filename)
        self.intrinsic_mat = self.get_intrinsic()
        self.intrinsic_mat_inv = np.linalg.inv(self.intrinsic_mat)

        img_pixs = np.mgrid[
            0 : self.cfg_data["Camera.height"] : subsample_pixs,
            0 : self.cfg_data["Camera.width"] : subsample_pixs,
        ].reshape(2, -1)
        img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
        self.uv_one = np.concatenate((img_pixs, np.ones((1, img_pixs.shape[1]))))
        self.uv_one_in_cam = np.dot(self.intrinsic_mat_inv, self.uv_one)

    def get_pcd_ic(self, depth_im, rgb_im=None):
        """
        Returns the point cloud in camera's coordinate frame

        :param depth_im: depth image (shape: :math:`[H, W]`)
        :param rgb_im: rgb image (shape: :math:`[H, W, 3]`)

        :type depth_im: numpy.ndarray
        :type rgb_im: numpy.ndarray

        :returns: tuple (pts_in_cam, rgb_im)
                  pts_in_cam: point coordinates in
                              camera frame (shape: :math:`[4, N]`)
                  rgb: rgb values for pts_in_cam (shape: :math:`[N, 3]`)
        :rtype (numpy.ndarray, numpy.ndarray)
        """
        # pcd in camera from depth
        depth_im = depth_im[0 :: self.subsample_pixs, 0 :: self.subsample_pixs]
        rgb_im = rgb_im[0 :: self.subsample_pixs, 0 :: self.subsample_pixs]
        depth = depth_im.reshape(-1) / float(self.cfg_data["DepthMapFactor"])
        rgb = None
        if rgb_im is not None:
            rgb = rgb_im.reshape(-1, 3)
        if self.depth_threshold is not None:
            valid = depth > self.depth_threshold[0]
            if len(self.depth_threshold) > 1:
                valid = np.logical_and(valid, depth < self.depth_threshold[1])
            uv_one_in_cam = self.uv_one_in_cam[:, valid]
            depth = depth[valid]
            rgb = rgb[valid]
        else:
            uv_one_in_cam = self.uv_one_in_cam
        pts_in_cam = np.multiply(uv_one_in_cam, depth)
        pts_in_cam = np.concatenate(
            (pts_in_cam, np.ones((1, pts_in_cam.shape[1]))), axis=0
        )
        return pts_in_cam, rgb

    def get_pcd_iw(self, pts_in_cam, extrinsic_mat):
        """
        Returns the point cloud in the world coordinate frame

        :param pts_in_cam: point coordinates in
               camera frame (shape: :math:`[4, N]`)
        :param extrinsic_mat: extrinsic matrix for
               the camera (shape: :math:`[4, 4]`)

        :type pts_in_cam: numpy.ndarray
        :type extrinsic_mat: numpy.ndarray

        :return: point coordinates in
                 ORB-SLAM2's world frame (shape: :math:`[N, 3]`)
        :rtype: numpy.ndarray
        """
        # pcd in world
        pts_in_world = np.dot(extrinsic_mat, pts_in_cam)
        pts_in_world = pts_in_world[:3, :].T
        return pts_in_world

    def read_cfg(self, cfg_filename):
        """
        Reads the configuration file

        :param cfg_filename: configuration file name for ORB-SLAM2

        :type cfg_filename: string

        :return: configurations in the configuration file
        :rtype: dict
        """
        rospack = rospkg.RosPack()
        slam_pkg_path = rospack.get_path("orb_slam2_ros")
        cfg_path = os.path.join(slam_pkg_path, "cfg", cfg_filename)
        with open(cfg_path, "r") as f:
            for i in range(1):
                f.readline()
            cfg_data = yaml.load(f)
        return cfg_data

    def get_intrinsic(self):
        """
        Returns the instrinsic matrix of the camera

        :return: the intrinsic matrix (shape: :math:`[3, 3]`)
        :rtype: numpy.ndarray
        """
        fx = self.cfg_data["Camera.fx"]
        fy = self.cfg_data["Camera.fy"]
        cx = self.cfg_data["Camera.cx"]
        cy = self.cfg_data["Camera.cy"]
        Itc = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        return Itc


class PointCloudProcessor:
    """
    This class subscribes to the pose published by ORB-SLAM2,
    reads the corresponding RGB and depth images, and generate
    the point cloud
    """

    def __init__(
        self,
        rgb_dir,
        depth_dir,
        cfg_filename,
        subsample_pixs=1,
        depth_threshold=(0, 1.5),
        camera_traj_topic="/orb_slam2_rgbd/slam/camera_traj",
    ):
        """
        The constructor for :class:`PointCloudProcessor` class.

        :param rgb_dir: directory of the RGB images
        :param depth_dir: directory of the depth images
        :param cfg_filename: configuration file name for ORB-SLAM2
        :param subsample_pixs: sample rows and columns for the images
        :param depth_threshold: minimum and maximum of valid depth values

        :type rgb_dir: string
        :type depth_dir: string
        :type cfg_filename: string
        :type subsample_pixs: int
        :type depth_threshold: tuple
        """
        if not USE_ORB_SLAM2:
            raise ValueError("ORB-SLAM2 not installed! Cannot fetch robot pose!")
        self.rgb_dir = rgb_dir
        self.depth_dir = depth_dir

        self.pcd_pool_in_cam = {}  # {keyframe id: pts_in_cam}
        self.extrinsic_mats = {}  # {keyframe id: extrinsic mat}
        self.depth_cam = DepthImgProcessor(
            subsample_pixs=subsample_pixs,
            depth_threshold=depth_threshold,
            cfg_filename=cfg_filename,
        )
        self.lock = threading.Lock()
        rospy.Subscriber(camera_traj_topic, Traj, self._traj_callback, queue_size=1)

    def get_current_pcd(self):
        """
        Returns the point cloud seen so far

        :returns: tuple (all_pts, all_colors)
                  all_pts: point coordinates in
                           ORB-SLAM2's world frame (shape: :math:`[N, 3]`)
                  all_colors: rgb values for pts_in_cam (shape: :math:`[N, 3]`)
        :rtype (numpy.ndarray, numpy.ndarray)
        """
        self.lock.acquire()
        if len(self.extrinsic_mats) < 1:
            rospy.loginfo(
                "No camera trajectory data is received,\n"
                "Please check if ORB-SLAM2 is running properly."
            )
            return None, None
        num_frames = 0
        num_pts = 0
        for KFid, ext_mat in self.extrinsic_mats.iteritems():
            if KFid not in self.pcd_pool_in_cam:
                continue
            num_frames += 1
            pcd_in_cam, rgb = self.pcd_pool_in_cam[KFid]
            num_pts += pcd_in_cam.shape[1]
        # rospy.loginfo('Number of frames to be stitched: %d' % num_frames)
        # rospy.loginfo('Number of points: %d' % num_pts)
        all_pts = np.zeros((num_pts, 3))
        all_colors = np.zeros((num_pts, 3))
        cur_id = 0
        for KFid, ext_mat in self.extrinsic_mats.iteritems():
            if KFid not in self.pcd_pool_in_cam:
                continue
            pcd_in_cam, rgb = self.pcd_pool_in_cam[KFid]
            pcd_in_world = self.depth_cam.get_pcd_iw(pcd_in_cam, ext_mat)
            all_pts[cur_id : cur_id + pcd_in_world.shape[0], :] = pcd_in_world
            all_colors[cur_id : cur_id + pcd_in_world.shape[0], :] = rgb
            cur_id += pcd_in_world.shape[0]
        self.lock.release()
        return all_pts, all_colors

    def _traj_callback(self, traj_data):
        """
        Callback function for the trajectory subscriber,
        it saves all the points (in their own camera coordinate frame) in memory

        :param traj_data: trajectory data
        :type traj_data: orb_slam2_ros.msg.Traj
        """
        KFids = traj_data.mnIds
        KFposes = traj_data.poses
        num_KFs = len(KFids)
        self.lock.acquire()
        for i in range(num_KFs):
            KFid = KFids[i]
            KFpose = KFposes[i]
            extrinsic_mat = self.cam_pose_to_ext_mat(KFpose)
            self.extrinsic_mats[KFid] = extrinsic_mat
            if KFid not in self.pcd_pool_in_cam:
                im_filename = "KeyFrame-{0:08d}.png".format(KFid)
                rgb_im_path = os.path.join(self.rgb_dir, im_filename)
                depth_im_path = os.path.join(self.depth_dir, im_filename)
                depth_im = cv2.imread(depth_im_path, cv2.IMREAD_UNCHANGED)
                rgb_im = cv2.imread(rgb_im_path)
                if depth_im is None:
                    continue
                rgb_im = rgb_im[:, :, ::-1]
                pcd_in_cam, rgb = self.depth_cam.get_pcd_ic(
                    depth_im=depth_im, rgb_im=rgb_im
                )
                self.pcd_pool_in_cam[KFid] = (pcd_in_cam, rgb)
        self.lock.release()

    def cam_pose_to_ext_mat(self, pose):
        """
        Convert the ros topic pose (geometry_msgs/Pose) into the extrinsic matrix

        :param pose: pose
        :type pose: geometry_msgs.Pose
        :return: extrinsic matrix (shape: :math:`[4, 4]`)
        :rtype: numpy.ndarray
        """
        trans = np.array([pose.position.x, pose.position.y, pose.position.z])
        rot_quat = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        rot_mat = tf.transformations.quaternion_matrix(quaternion=rot_quat)[:3, :3]
        E = np.eye(4)
        E[:3, :3] = rot_mat
        E[:3, 3] = trans
        return E


def main():
    home_dir = expanduser("~")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--img_dir",
        help="path to the directory that saves" " depth and rgb images from ORB_SLAMA2",
        default="%s/.ros/Imgs" % home_dir,
        type=str,
    )
    parser.add_argument(
        "--depth_max", default=1.5, help="maximum value for the depth", type=float
    )
    parser.add_argument(
        "--depth_min", default=0.2, help="minimum value for the depth", type=float
    )
    parser.add_argument(
        "--cfg_filename",
        default="realsense_d435.yaml",
        help="which config file to use",
        type=str,
    )
    parser.add_argument(
        "--subsample_pixs",
        default=1,
        help="sample every n pixels in row/column"
        "when the point cloud is reconstructed",
        type=int,
    )
    args = parser.parse_args()
    rospy.init_node("reconstruct_world", anonymous=True)
    rgb_dir = os.path.join(args.img_dir, "RGBImgs")
    depth_dir = os.path.join(args.img_dir, "DepthImgs")
    pcd_processor = PointCloudProcessor(
        rgb_dir=rgb_dir,
        depth_dir=depth_dir,
        cfg_filename=args.cfg_filename,
        subsample_pixs=args.subsample_pixs,
        depth_threshold=(args.depth_min, args.depth_max),
    )
    time.sleep(2)
    points, colors = pcd_processor.get_current_pcd()
    if USE_OPEN3D:
        pcd = open3d.PointCloud()
        pcd.points = open3d.Vector3dVector(points)
        pcd.colors = open3d.Vector3dVector(colors / 255.0)
        coord = open3d.create_mesh_coordinate_frame(1, [0, 0, 0])
        open3d.draw_geometries([pcd, coord])


if __name__ == "__main__":
    main()
