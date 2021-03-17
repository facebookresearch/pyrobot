# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import rospy
from locobot_control.analytic_ik import AnalyticInverseKinematics as AIK
from locobot_control.srv import JointCommand
from pyrobot.robots.ros_arm import ROSArm
from std_msgs.msg import Empty

from pyrobot.utils.util import append_namespace


class LoCoBotArm(ROSArm):
    """
    This class has functionality to control a robotic arm
    in joint and task space (with and
    without any motion planning), for position/velocity/torque control, etc.
    """

    def __init__(self, configs, ns=""):
        """
        The constructor for LoCoBotArm class.

        :param configs: configurations read from config file
        :param control_mode: Choose between 'position',
                            'velocity' and 'torque' control
        :param moveit_planner: Planner name for moveit,
                               only used if planning_mode = 'moveit'.
        :param use_moveit: use moveit or not, default is True
        :type configs: YACS CfgNode
        :type control_mode: string
        :type moveit_planner: string
        :type use_moveit: bool
        """
        use_arm = rospy.get_param("use_arm", False)
        use_sim = rospy.get_param("use_sim", False)
        use_arm = use_arm or use_sim
        if not use_arm:
            rospy.logwarn(
                "Neither use_arm, nor use_sim, is not set to "
                "True when the LoCoBot driver is launched."
                "You may not be able to command the "
                "arm correctly using PyRobot!!!"
            )
            return
        super(LoCoBotArm, self).__init__(configs=configs, ns=ns)

        self.joint_stop_pub = rospy.Publisher(
            append_namespace(self.ns, self.configs.ROSTOPIC_STOP_EXECUTION),
            Empty,
            queue_size=1,
        )
        # Services
        self.joint_cmd_srv = rospy.ServiceProxy(
            append_namespace(self.ns, self.configs.ROSSERVICE_JOINT_COMMAND),
            JointCommand,
        )

        self.torque_cmd_srv = rospy.ServiceProxy(
            append_namespace(self.ns, self.configs.ROSTOPIC_TORQUE_COMMAND),
            JointCommand,
        )

    def set_joint_velocities(self, velocities, **kwargs):
        raise NotImplementedError("Velocity control for " "locobot not supported yet!")

    def set_joint_torque(self, joint_name, value):
        """

        :param joint_name: joint name (['joint_1',
                          'joint_2', 'joint_3', 'joint_4''])
        :param value: torque value in Nm
        :type joint_name: string
        :type value: float
        :return: sucessful or not
        :rtype: bool
        """
        joint_id_dict = {"joint_1": 1, "joint_2": 2, "joint_3": 3, "joint_4": 4}
        if joint_name in joint_id_dict:
            return self.torque_cmd_srv("newt", joint_id_dict[joint_name], value)
        else:
            rospy.logerr(
                "{} joint name provided, it should be one of this {}".format(
                    joint_name, sorted(joint_id_dict.keys())
                )
            )
            return False

    def set_joint_torques(self, torques, **kwargs):
        """
        Sets the desired joint torques for all arm joints.

        :param torques: target joint torques, list of len 4 populated
                        with torque to be applied on first 4 joints
                        of arm in Nm
        :type torques: list
        """
        if len(torques) == 4:
            joint_id_list = ["joint_1", "joint_2", "joint_3", "joint_4"]
            for index, value in enumerate(torques):
                self.set_joint_torque(joint_id_list[index], value)
        else:
            rospy.logerr("It is expecting input of type list of length 4")

    def go_home(self, plan=False):
        """
        Commands robot to home position

        :param plan: use moveit to plan the path or not
        :type plan: bool
        """
        return self.set_joint_positions(np.zeros(self.arm_dof), plan=plan)
