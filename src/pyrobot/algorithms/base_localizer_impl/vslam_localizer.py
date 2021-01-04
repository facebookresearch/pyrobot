from pyrobot.algorithms.base_localizer import BaseLocalizer

import rospy
import tf

import copy

from orb_slam2_ros.vslam import VisualSLAM

import numpy as np


class VSLAMLocalizer(BaseLocalizer):
    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):
        super(BaseLocalizer, self).__init__(
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

        self.vslam = VisualSLAM(
            map_img_dir=self.configs.MAP_IMG_DIR,
            cam_pose_tp=self.configs.ROSTOPIC_CAMERA_POSE,
            cam_traj_tp=self.configs.ROSTOPIC_CAMERA_TRAJ,
            base_frame=self.bot_base.configs.BASE_FRAME,
            camera_frame=self.configs.RGB_CAMERA_CENTER_FRAME,
            occ_map_rate=self.configs.OCCUPANCY_MAP_RATE,
            z_min=self.configs.Z_MIN_TRESHOLD_OCC_MAP,
            z_max=self.configs.Z_MAX_TRESHOLD_OCC_MAP,
        )

    def get_odom_state(self):
        return self.vslam.base_pose.copy()

    def check_cfg(self):
        super().check_cfg()

        assert "MAP_IMG_DIR" in self.configs.keys()
        assert "ROSTOPIC_CAMERA_POSE" in self.configs.keys()
        assert "ROSTOPIC_CAMERA_TRAJ" in self.configs.keys()
        assert "RGB_CAMERA_CENTER_FRAME" in self.configs.keys()
        assert "OCCUPANCY_MAP_RATE" in self.configs.keys()
        assert "Z_MIN_TRESHOLD_OCC_MAP" in self.configs.keys()
        assert "Z_MAX_TRESHOLD_OCC_MAP" in self.configs.keys()
