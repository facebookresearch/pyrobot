from pyrobot.algorithms.motion_planning import MotionPlanner

from moveit_pybind import Plan, MoveGroupInterfaceWrapper, Pose

import pyrobot.utils.util as prutil
import numpy as np


class MoveitPlanner(MotionPlanner):
    """
    Implementation of moveit!-based motion planning algorithm.
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
        self.move_group_interface.setMaxVelocityScalingFactor(0.3)
        self.ee_frame = self.robots[self.robot_label]["arm"].configs["EE_FRAME"]

    def plan_end_effector_pose(self, position, orientation):
        self.move_group_interface.clearPoseTargets()
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

        p_in = Pose()
        p_in.position.x = position[0]
        p_in.position.y = position[1]
        p_in.position.z = position[2]
        p_in.orientation.x = ori_x
        p_in.orientation.y = ori_y
        p_in.orientation.z = ori_z
        p_in.orientation.w = ori_w

        success = self.move_group_interface.setPoseTarget(p_in, self.ee_frame)
        p_out = self.move_group_interface.plan()
        self.move_group_interface.execute(p_out)

        return p_out.trajectory.joint_trajectory

    def plan_joint_angles(self, target_joint):
        self.move_group_interface.clearPoseTargets()
        target_joint = np.array(target_joint).flatten().tolist()
        success = self.move_group_interface.setJointValueTarget(target_joint)

        p_out = self.move_group_interface.plan()
        
        self.move_group_interface.execute(p_out)
        return p_out.trajectory.joint_trajectory

    def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
        raise NotImplementedError()

    def check_cfg(self):
        super().check_cfg()
