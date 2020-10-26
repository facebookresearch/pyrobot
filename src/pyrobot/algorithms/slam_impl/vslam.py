from pyrobot.algorithms.slam import SLAM

import rospy
import tf

import copy

from orb_slam2_ros.vslam import VisualSLAM

import numpy as np


class VSLAM(BaseLocalizer):

    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):
        super(SLAM, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )
        self.configs = configs
        self.state = [0.0, 0.0, 0.0]

        self.robot_label = list(self.robots.keys())[0]

        self.bot_base = self.robots[self.robot_label]["base"]

        self.bot_cam = self.robots[self.robot_label]["camera"]

        self.vslam = VisualSLAM(
            map_img_dir=self.configs.MAP_IMG_DIR,
            cam_pose_tp=self.configs.ROSTOPIC_CAMERA_POSE,
            cam_traj_tp=self.configs.ROSTOPIC_CAMERA_TRAJ,
            base_frame=self.bot_base.configs.BASE_FRAME,
            camera_frame=self.bot_cam.configs.RGB_CAMERA_CENTER_FRAME,
            occ_map_rate=self.configs.OCCUPANCY_MAP_RATE,
            z_min=self.configs.Z_MIN_TRESHOLD_OCC_MAP,
            z_max=self.configs.Z_MAX_TRESHOLD_OCC_MAP,
        )

    def get_odom_state(self):
        return self.vslam.base_pose.copy()

    def get_pointcloud_map(self):
        return self.vslam.get_3d_map()

    def get_occupancy_map(self):
        occ_map, xcells, ycells, x_min, y_min = self.vslam.get_occupancy_map()
        return occ_map

    def check_cfg(self):
        super().check_cfg()

        assert "MAP_IMG_DIR" in self.configs.keys()
        assert "ROSTOPIC_CAMERA_POSE" in self.configs.keys()
        assert "ROSTOPIC_CAMERA_TRAJ" in self.configs.keys()
        assert "OCCUPANCY_MAP_RATE" in self.configs.keys()
        assert "Z_MIN_TRESHOLD_OCC_MAP" in self.configs.keys()
        assert "Z_MAX_TRESHOLD_OCC_MAP" in self.configs.keys()
