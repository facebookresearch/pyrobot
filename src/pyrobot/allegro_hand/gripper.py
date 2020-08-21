# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import numpy as np
import rospy
import threading
import time

from pyrobot.core import Gripper
from std_msgs.msg import String as str_msg
from sensor_msgs.msg import JointState


class AllegroHand(Gripper):
    """
    This class has functionality to control a UR5 manipulator.
    """

    def __init__(self, configs):
        """
        The constructor is for Gripper class.

        :param configs: configurations read from config file
        :param moveit_planner: Planner name for moveit,
                               only used if planning_mode = 'moveit'.
        :type configs: YACS CfgNode
        :type moveit_planner: string
        """
        super(AllegroHand, self).__init__(configs=configs)

        # Publishers
        self.controller = rospy.get_param(self.configs.GRIPPER.ROSPARAM_CONTROLLER)

        if self.controller == "grasp":
            self.joint_pub = None
            self._setup_joint_pub()
        elif self.controller == "torque":
            self.torque_pub = None
            self._setup_torque_pub()
        else:
            RuntimeError("The only supported controllers are grasp and torque")

        self.primitive_pub = None
        self.primitives = self.configs.GRIPPER.PRIMITIVES
        self._setup_primitive_pub()

        self.gripper_joint_names = self.configs.GRIPPER.JOINT_NAMES
        self.gripper_dof = len(self.gripper_joint_names)

        # Subscribers
        self.joint_state_lock = threading.RLock()
        self._joint_angles = dict()
        self._joint_velocities = dict()
        self._joint_efforts = dict()
        rospy.Subscriber(
            configs.GRIPPER.ROSTOPIC_JOINT_STATES,
            JointState,
            self._callback_joint_states,
        )

    def set_primitive(self, primitive, wait=True):
        if primitive not in self.primitives:
            ValueError(
                "Invalid primitive. Only the following are available: {}".format(
                    self.primitives
                )
            )
        msg = str_msg()
        msg.data = primitive
        self.primitive_pub.publish(msg)
        if wait:
            rospy.sleep(self.configs.GRIPPER.WAIT_MIN_TIME)

    def go_home(self):
        """
        Commands robot to home position
        """
        self.set_joint_positions(np.zeros(self.gripper_dof), plan=False)

    def move_to_neutral(self):
        """
        Move the robot to a pre-defined neutral pose
        """

        # TODO: Change it to some better neutral position
        neutral_pos = np.asarray(
            [
                -0.047094788747171,
                0.12041967890287064,
                0.9031856318825505,
                0.7667246220374792,
                -0.000653114836499732,
                0.1256168065087617,
                0.8983287641635962,
                0.811247063242525,
                0.019528077323928285,
                0.1802489201211747,
                1.0038365983813113,
                0.8052881420910417,
                1.1071484265259601,
                0.6201480040823202,
                0.2677506665261866,
                0.885150694760694,
            ]
        )
        self.set_joint_positions(neutral_pos, plan=False)

    def set_joint_velocities(self, velocities, **kwargs):
        raise NotImplementedError(
            "Velocity control for " "Allegro Hand is not supported yet!"
        )

    def set_joint_positions(self, positions, plan=False, wait=True, **kwargs):
        """
        Sets the desired joint angles for all gripper joints

        :param positions: list of length #of joints, angles in radians
        :param plan: whether to use moveit to plan a path. Without planning,
                     there is no collision checking and each joint will
                     move to its target joint position directly.
        :param wait: if True, it will wait until the desired
                     joint positions are achieved
        :type positions: list
        :type plan: bool
        :type wait: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        if self.controller is not "grasp":
            RuntimeError("Please choose grasp controller to allow setting positions")

        result = False
        if isinstance(positions, np.ndarray):
            positions = positions.flatten().tolist()
        if plan:
            raise NotImplementedError(
                "Using planning while gripper position control\
                                        is not supported"
            )
        else:
            self._pub_joint_positions(positions)
            if wait:
                rospy.sleep(self.configs.GRIPPER.WAIT_MIN_TIME)
            res = self._angle_error_is_small(positions)
            joint_diff, error, eflag = res
            if eflag:
                result = True
        return result

    def get_joint_angles(self):
        """
        Return the joint angles

        :return: joint_angles
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        joint_angles = []
        for joint in self.gripper_joint_names:
            joint_angles.append(self.get_joint_angle(joint))
        joint_angles = np.array(joint_angles).flatten()
        self.joint_state_lock.release()
        return joint_angles

    def get_joint_velocities(self):
        """
        Return the joint velocities

        :return: joint_vels
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        joint_vels = []
        for joint in self.gripper_joint_names:
            joint_vels.append(self.get_joint_velocity(joint))
        joint_vels = np.array(joint_vels).flatten()
        self.joint_state_lock.release()
        return joint_vels

    def get_joint_torques(self):
        """
        Return the joint torques

        :return: joint_torques
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        joint_torques = []
        for joint in self.gripper_joint_names:
            try:
                joint_torques.append(self.get_joint_torque(joint))
            except (ValueError, IndexError):
                rospy.loginfo("Torque value for joint " "[%s] not available!" % joint)
        joint_torques = np.array(joint_torques).flatten()
        self.joint_state_lock.release()
        return joint_torques

    def get_joint_angle(self, joint):
        """
        Return the joint angle of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint angle
        :rtype: float
        """
        if joint not in self.gripper_joint_names:
            raise ValueError("%s not in gripper joint list!" % joint)
        if joint not in self._joint_angles.keys():
            raise ValueError("Joint angle for joint $s not available!" % joint)
        return self._joint_angles[joint]

    def get_joint_velocity(self, joint):
        """
        Return the joint velocity of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint velocity
        :rtype: float
        """
        if joint not in self.gripper_joint_names:
            raise ValueError("%s not in gripper joint list!" % joint)
        if joint not in self._joint_velocities.keys():
            raise ValueError("Joint velocity for joint" " $s not available!" % joint)
        return self._joint_velocities[joint]

    def get_joint_torque(self, joint):
        """
        Return the joint torque of the 'joint'

        :param joint: joint name
        :type joint: string
        :return: joint torque
        :rtype: float
        """
        if joint not in self.gripper_joint_names:
            raise ValueError("%s not in gripper joint list!" % joint)
        if joint not in self._joint_efforts.keys():
            raise ValueError("Joint torque for joint $s" " not available!" % joint)
        return self._joint_efforts[joint]

    def set_joint_torques(self, torques, **kwargs):
        """
        Sets the desired joint torques for all gripper joints

        :param torques: target joint torques
        :type torques: list
        """
        if self.controller is not "torque":
            RuntimeError("Please choose torque controller to allow setting torques")
        self._pub_joint_torques(torques)

    def _pub_joint_torques(self, torques):
        joint_state = JointState()
        joint_state.effort = tuple(torques)
        self.torque_pub.publish(joint_state)

    def _callback_joint_states(self, msg):
        """
        ROS subscriber callback for gripper joint state (position, velocity)

        :param msg: Contains message published in topic
        :type msg: sensor_msgs/JointState
        """
        self.joint_state_lock.acquire()
        for idx, name in enumerate(msg.name):
            if name in self.gripper_joint_names:
                if idx < len(msg.position):
                    self._joint_angles[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._joint_velocities[name] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self._joint_efforts[name] = msg.effort[idx]
        self.joint_state_lock.release()

    def _pub_joint_positions(self, positions):
        joint_state = JointState()
        joint_state.position = tuple(positions)
        self.joint_pub.publish(joint_state)

    def _angle_error_is_small(self, target_joints):
        cur_joint_vals = self.get_joint_angles()
        joint_diff = cur_joint_vals - np.array(target_joints)
        error = np.max(np.abs(joint_diff))
        if error <= self.configs.GRIPPER.MAX_JOINT_ERROR:
            return joint_diff, error, True
        else:
            return joint_diff, error, False

    def _setup_joint_pub(self):
        self.joint_pub = rospy.Publisher(
            self.configs.GRIPPER.ROSTOPIC_SET_JOINT, JointState, queue_size=1
        )

    def _setup_torque_pub(self):
        self.torque_pub = rospy.Publisher(
            self.configs.GRIPPER.ROSTOPIC_SET_TORQUE, JointState, queue_size=1
        )

    def _setup_primitive_pub(self):
        self.primitive_pub = rospy.Publisher(
            self.configs.GRIPPER.ROSTOPIC_SET_PRIMITIVE, str_msg, queue_size=1
        )

    def open(self, **kwargs):
        self.set_primitive("home")

    def close(self, **kwargs):
        self.set_primitive("envelop")
