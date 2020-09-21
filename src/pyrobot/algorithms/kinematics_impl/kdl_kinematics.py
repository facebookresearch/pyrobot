from pyrobot.algorithms.kinematics import Kinematics

from .kdl_solver import KDLSolver

import pyrobot.utils.util as prutil
import numpy as np


class KDLKinematics(Kinematics):
    """Implementation of KDL-based Kinematics algorithms.
    Specifically, forward/inverse kinematics, and jacobian.
    """

    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):

        super(Kinematics, self).__init__(
            configs, world, ros_launch_manager, robots, sensors, algorithms
        )

        self.robot_label = list(self.robots.keys())[0]

        self.kdl_solver = KDLSolver(
            self.robots[self.robot_label]["arm"].configs["ARM_BASE_FRAME"],
            self.robots[self.robot_label]["arm"].configs["EE_FRAME"],
            self.robots[self.robot_label]["arm"].configs["ARM_ROBOT_DSP_PARAM_NAME"],
        )

    def inverse_kinematics(self, position, orientation, init_joint_pos=None):
        position = np.array(position).flatten()
        orientation = np.array(orientation).flatten()

        if orientation.size == 4:
            orientation = orientation.flatten()
            ori_x = orientation[0]
            ori_y = orientation[1]
            ori_z = orientation[2]
            ori_w = orientation[3]
        elif orientation.size == 3:
            quat = prutil.euler_to_quat(orientation)
            ori_x = quat[0]
            ori_y = quat[1]
            ori_z = quat[2]
            ori_w = quat[3]
        elif orientation.size == 9:
            orientation = orientation.reshape((3, 3))
            quat = prutil.rot_mat_to_quat(orientation)
            ori_x = quat[0]
            ori_y = quat[1]
            ori_z = quat[2]
            ori_w = quat[3]
        else:
            raise TypeError(
                "Orientation must be in one "
                "of the following forms:"
                "rotation matrix, euler angles, or quaternion"
            )

        if init_joint_pos is None:
            init_joint_pos = self.robots[self.robot_label]["arm"].get_joint_angles()
        init_joint_pos = np.array(init_joint_pos).flatten().tolist()

        pos_tol = self.configs.IK_POSITION_TOLERANCE
        ori_tol = self.configs.IK_ORIENTATION_TOLERANCE

        pose = [
            position[0],
            position[1],
            position[2],
            ori_x,
            ori_y,
            ori_z,
            ori_w,
        ]

        tolerance = 3 * [pos_tol] + 3 * [ori_tol]
        joint_positions = self.kdl_solver.ik(pose, tolerance, init_joint_pos)
        return joint_positions

    def forward_kinematics(self, joint_pos, target_frame):
        pos, quat = self.kdl_solver.fk(joint_positions, des_frame)
        pos = np.asarray(pos).reshape(3, 1)
        rot = prutil.quat_to_rot_mat(quat)
        return pos, rot

    def check_cfg(self):
        assert (
            len(self.robots.keys()) == 1
        ), "One Kinematics solver only handle one arm!"
        robot_label = list(self.robots.keys())[0]
        assert "arm" in self.robots[robot_label].keys(), "Arm required for kinematics!"
        assert (
            "ARM_BASE_FRAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "ARM_BASE_FRAME required for KDL solver!"
        assert (
            "EE_FRAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "EE_FRAME required for KDL solver!"
        assert (
            "ARM_ROBOT_DSP_PARAM_NAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "ARM_ROBOT_DSP_PARAM_NAME required for KDL solver!"
        assert "IK_POSITION_TOLERANCE" in self.configs.keys()
        assert "IK_ORIENTATION_TOLERANCE" in self.configs.keys()
