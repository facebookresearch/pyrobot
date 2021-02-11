from pyrobot.algorithms.camera_transform import CameraTransform
from pyrobot.algorithms.frame_transform import FrameTransform

import pyrobot.utils.util as prutil

import numpy as np


class DefaultCameraTransform(CameraTransform):
    """base class of camera transformation algorithms."""

    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):
        super(CameraTransform, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )
        if len(self.robots.keys()) > 0:
            self.robot_label = list(self.robots.keys())[0]
            self.camera = self.robots[self.robot_label]["camera"]
        else:
            self.sensor_label = list(self.sensors.keys())[0]
            self.camera = self.sensors[self.sensor_label]

        self.base_frame = self.camera.configs["BASE_FRAME"]
        self.cam_frame = self.camera.configs["RGB_CAMERA_CENTER_FRAME"]

        self.depth_map_factor = self.configs["DEPTH_MAP_FACTOR"]
        self.subsample_pixs = self.configs["SUBSAMPLE_PIXS"]

        self.depth_min = self.configs["DEPTH_MIN"]
        self.depth_max = self.configs["DEPTH_MAX"]

        self.intrinsic_mat_inv = np.linalg.inv(self.get_intrinsics())

        img_pixs = np.mgrid[
            0 : self.camera.configs["Camera.height"] : self.subsample_pixs,
            0 : self.camera.configs["Camera.width"] : self.subsample_pixs,
        ]
        img_pixs = img_pixs.reshape(2, -1)
        img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
        self.uv_one = np.concatenate((img_pixs, np.ones((1, img_pixs.shape[1]))))
        self.uv_one_in_cam = np.dot(self.intrinsic_mat_inv, self.uv_one)

    def pix_to_pt(self, rows, columns, depth_img, rgb_img=None, in_cam=True):
        trans, quat = self.algorithms["FrameTransform"].get_transform(
            self.cam_frame, self.base_frame
        )
        rot = prutil.quat_to_rot_mat(quat)
        base2cam_trans = np.array(trans).reshape(-1, 1)
        base2cam_rot = np.array(rot)

        pts_in_cam = prutil.pix_to_3dpt(
            depth_img,
            rows,
            columns,
            self.get_intrinsics(),
            float(self.depth_map_factor),
        )
        pts = pts_in_cam[:3, :].T

        colors = None
        if rgb_img is not None:
            colors = rgb_img[rs, cs].reshape(-1, 3)

        if in_cam:
            return pts, colors

        pts = np.dot(pts, base2cam_rot.T)
        pts = pts + base2cam_trans.T
        return pts, colors

    def pcd_from_img(self, depth_img, rgb_img=None, in_cam=True):
        trans, quat = self.algorithms["FrameTransform"].get_transform(
            self.cam_frame, self.base_frame
        )
        rot = prutil.quat_to_rot_mat(quat)
        base2cam_trans = np.array(trans).reshape(-1, 1)
        base2cam_rot = np.array(rot)

        depth_img = depth_img[0 :: self.subsample_pixs, 0 :: self.subsample_pixs]
        if rgb_img:
            rgb_img = rgb_img[0 :: self.subsample_pixs, 0 :: self.subsample_pixs]
        depth = depth_img.reshape(-1) / float(self.depth_map_factor)
        rgb = None
        if rgb_img is not None:
            rgb = rgb_img.reshape(-1, 3)
        if self.depth_min and self.depth_max is not None:
            valid = depth > self.depth_min
            valid = np.logical_and(valid, depth < self.depth_max)
            uv_one_in_cam = self.uv_one_in_cam[:, valid]
            depth = depth[valid]
            if rgb is not None:
                rgb = rgb[valid]
        else:
            uv_one_in_cam = self.uv_one_in_cam
        pts_in_cam = np.multiply(uv_one_in_cam, depth)
        pts_in_cam = np.concatenate(
            (pts_in_cam, np.ones((1, pts_in_cam.shape[1]))), axis=0
        )

        pts = pts_in_cam[:3, :].T
        if in_cam:
            return pts, rgb
        pts = np.dot(pts, base2cam_rot.T)
        pts = pts + base2cam_trans.T
        return pts, rgb

    def get_transform_matrix(self, source_frame, target_frame):
        trans, quat = self.algorithms["FrameTransform"].get_transform(
            source_frame, target_frame
        )
        rot = prutil.quat_to_rot_mat(quat)
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = trans
        return T

    def get_intrinsics(self):
        fx = self.camera.configs["Camera.fx"]
        fy = self.camera.configs["Camera.fy"]
        cx = self.camera.configs["Camera.cx"]
        cy = self.camera.configs["Camera.cy"]
        Itc = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        return Itc

    def check_cfg(self):
        super().check_cfg()

        assert (
            len(self.algorithms.keys()) == 1
        ), "CameraTransform only have one dependency!"

        assert (
            list(self.algorithms.keys())[0] == "FrameTransform"
        ), "CameraTransform only depend on FrameTransform!"

        assert isinstance(
            self.algorithms["FrameTransform"], FrameTransform
        ), "FrameTransform module needs to extend FrameTransform base class!"

        assert (
            "SUBSAMPLE_PIXS" in self.configs.keys()
        ), "SUBSAMPLE_PIXS required for DefaultCameraTransform!"

        assert (
            "DEPTH_MIN" in self.configs.keys()
        ), "DEPTH_MIN required for DefaultCameraTransform!"

        assert (
            "DEPTH_MAX" in self.configs.keys()
        ), "DEPTH_MAX required for DefaultCameraTransform!"

        assert (
            "DEPTH_MAP_FACTOR" in self.configs.keys()
        ), "DEPTH_MAP_FACTOR required for DefaultCameraTransform!"
