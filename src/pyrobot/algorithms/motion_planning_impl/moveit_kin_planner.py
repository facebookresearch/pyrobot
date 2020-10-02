from pyrobot.algorithms.motion_planning import MotionPlanner
from pyrobot.algorithms.kinematics import Kinematics

from moveit_pybind import Plan, MoveGroupInterfaceWrapper, Pose

import pyrobot.utils.util as prutil
import numpy as np


class MoveitKinPlanner(MotionPlanner):
    """
    Implementation of moveit!-based motion planning algorithm, with explicit kinematics dependency.
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

        super(MotionPlanner, self).__init__(
            configs, world, ros_launch_manager, robots, sensors, algorithms
        )

        self.robot_label = list(self.robots.keys())[0]

        self.move_group_interface = MoveGroupInterfaceWrapper(
            self.robots[self.robot_label]["arm"].configs["MOVEGROUP_NAME"],
            self.robots[self.robot_label]["arm"].configs["ARM_ROBOT_DSP_PARAM_NAME"],
        )
        self.ee_frame = self.robots[self.robot_label]["arm"].configs["EE_FRAME"]

    def plan_end_effector_pose(self, position, orientation):
        target_joint = self.algorithms["Kinematics"].inverse_kinematics(
            position, orientation
        )

        p_out = self.plan_joint_angles(target_joint)
        return p_out

    def plan_joint_angles(self, target_joint):
        self.move_group_interface.clearPoseTargets()
        target_joint = np.array(target_joint).flatten().tolist()
        success = self.move_group_interface.setJointValueTarget(target_joint)

        p_out = self.move_group_interface.plan()
        self.move_group_interface.execute(p_out)
        return p_out

    def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
        raise NotImplementedError()

    def check_cfg(self):
        super().check_cfg()

        assert (
            len(self.algorithms.keys()) == 1
        ), "MoveitKinPlanner only have one dependency!"
        assert (
            list(self.algorithms.keys())[0] == "Kinematics"
        ), "MoveitKinPlanner only depend on Kinematics!"
        assert isinstance(
            self.algorithms["Kinematics"], Kinematics
        ), "Kinematics module needs to extend Kinematics base class!"