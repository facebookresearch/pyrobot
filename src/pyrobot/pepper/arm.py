#!/usr/bin/env python
# coding: utf-8

import time
import rospy
import numpy as np
from pyrobot.core import Arm
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class PepperArm(Arm):
    """
    This class allows to control Pepper's arm in the joint task space
    """

    def __init__(self,
                 configs,
                 moveit_planner='ESTkConfigDefault',
                 use_moveit=False):
        """
        Constructor for the PepperArm class.

        :param configs: configurations read from config file
        :param moveit_planner: Planner name for moveit,
                               only used if planning_mode = 'moveit'.
        :param use_moveit: use moveit or not, default is True
        :type configs: YACS CfgNode
        :type moveit_planner: string
        :type use_moveit: bool
        """
        # Joint names for the left arm, considered as the extra arm
        self.extra_arm_joint_names = configs.ARM.EXTRA_JOINT_NAMES
        self.extra_arm_dof = len(self.extra_arm_joint_names)

        # Extra subscribers (for the left arm)
        self._extra_joint_angles = dict()
        self._extra_joint_velocities = dict()
        self._extra_joint_efforts = dict()

        rospy.set_param(
            'pyrobot/extra_gripper_link',
            configs.ARM.EXTRA_EE_FRAME)

        super(PepperArm, self).__init__(
            configs=configs,
            moveit_planner=moveit_planner,
            use_moveit=use_moveit)

    def go_home(self, r_arm=True, speed=0.6, wait=False):
        """
        Resets the robot to the default home position

        :param r_arm: use the right or left arm, default is right
        :param speed: The percentage of the max angular speed to be used for
                      the movement (between 1.0 and 0.0)
        :param wait: if True, it will wait until the desired
                     joint positions are achieved
        :type r_arm: bool
        :type speed: float
        :type wait: bool
        """
        self.set_joint_positions(
            np.zeros(self.arm_dof),
            r_arm=r_arm,
            plan=False,
            speed=speed,
            wait=wait)

    def move_to_neutral(self, r_arm=True, speed=0.6, wait=False):
        """
        Moves the arm of the robot to a neutral pose (normal standing position)

        :param r_arm: use the right or left arm, default is right
        :param speed: The percentage of the max angular speed to be used for
                      the movement (between 1.0 and 0.0)
        :param wait: if True, it will wait until the desired
                     joint positions are achieved
        :type r_arm: bool
        :type speed: float
        :type wait: bool
        """
        if r_arm:
            neutral_pos = [1.580, -0.115, 1.226, 0.518, 0.027]
        else:
            neutral_pos = [1.580, 0.115, -1.226, -0.518, -0.027]

        self.set_joint_positions(
            neutral_pos,
            r_arm=r_arm,
            plan=False,
            speed=speed,
            wait=wait)

    @property
    def pose_extra_ee(self):
        """
        Return the extra (left) end effector pose w.r.t 'ARM_BASE_FRAME'

        :return:
                trans: translational vector (shape: :math:`[3, 1]`)

                rot_mat: rotational matrix (shape: :math:`[3, 3]`)

                quat: rotational matrix in the form
                of quaternion (shape: :math:`[4,]`)

        :rtype: tuple (trans, rot_mat, quat)
        """
        return self.get_extra_ee_pose(
            base_frame=self.configs.ARM.ARM_BASE_FRAME)

    def get_extra_ee_pose(self, base_frame):
        """
        Return the extra (left) end effector pose with respect to the
        base_frame

        :param base_frame: reference frame
        :type base_frame: string
        :return:
                tuple (trans, rot_mat, quat)

                trans: translational vector (shape: :math:`[3, 1]`)

                rot_mat: rotational matrix (shape: :math:`[3, 3]`)

                quat: rotational matrix in the form
                of quaternion (shape: :math:`[4,]`)

        :rtype: tuple or ROS PoseStamped
        """
        return self.get_transform(base_frame, self.configs.ARM.EXTRA_EE_FRAME)

    def get_extra_joint_angles(self):
        """
        Return the extra joint angles (for the left arm)

        :return: extra_joint_angles
        :rtype: np.ndarray
        """
        self.joint_state_lock.acquire()
        extra_joint_angles = []
        for joint in self.extra_arm_joint_names:
            extra_joint_angles.append(self.get_extra_joint_angle(joint))
        extra_joint_angles = np.array(extra_joint_angles).flatten()
        self.joint_state_lock.release()
        return extra_joint_angles

    def get_extra_joint_angle(self, joint):
        """
        Return the joint angle of the 'joint' for the left arm

        :param joint: joint name
        :type joint: string
        :return: joint angle
        :rtype: float
        """
        if joint not in self.extra_arm_joint_names:
            raise ValueError('%s not in extra arm joint list!' % joint)
        if joint not in self._extra_joint_angles.keys():
            raise ValueError(
                'Extra joint angle for joint $s not available!' % joint)
        return self._extra_joint_angles[joint]

    def get_joint_velocity(self, joint):
        raise NotImplementedError(
            'Velocity getter for a Pepper joint is not implemented')

    def get_joint_torque(self, torque):
        raise NotImplementedError(
            'Torque getter for a Pepper joint is not implemented')

    def set_joint_positions(
            self,
            positions,
            r_arm=True,
            plan=False,
            speed=0.6,
            wait=False,
            **kwargs):
        """
        Sets the desired joint angles for all arm joints

        :param positions: list of length #of joints, angles in radians
        :param r_arm: use the right or left arm, default is right
        :param plan: whether to use moveit to plan a path. Without planning,
                     there is no collision checking and each joint will
                     move to its target joint position directly. False by
                     default
        :param speed: The percentage of the max angular speed to be used for
                      the movement (between 1.0 and 0.0)
        :param wait: if True, it will wait until the desired
                     joint positions are achieved. False by default
        :type positions: list
        :type r_arm: bool
        :type plan: bool
        :type speed: float
        :type wait: bool

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        result = False

        if isinstance(positions, np.ndarray):
            positions = positions.flatten().tolist()

        if plan:
            if not self.use_moveit:
                raise ValueError(
                    "Moveit is not initialized, did you pass in "
                    "use_moveit=True?")

            if isinstance(positions, np.ndarray):
                positions = positions.tolist()

            if r_arm:
                joint_names = self.arm_joint_names
            else:
                joint_names = self.extra_arm_joint_names

            rospy.loginfo('Moveit Motion Planning...')
            result = self.moveit_group.moveToJointPosition(
                joint_names,
                positions,
                wait=wait)
        else:
            if wait:
                result = self._loop_angle_pub_cmd(
                    self._pub_joint_positions,
                    positions,
                    speed=speed,
                    r_arm=r_arm)
            else:
                self._pub_joint_positions(positions, speed=speed, r_arm=r_arm)

        return result

    def _angle_error_is_small(self, target_joints, r_arm=True):
        if r_arm:
            cur_joint_vals = self.get_joint_angles()
        else:
            cur_joint_vals = self.get_extra_joint_angles()

        joint_diff = cur_joint_vals - np.array(target_joints)
        error = np.max(np.abs(joint_diff))
        if error <= self.configs.ARM.MAX_JOINT_ERROR:
            return joint_diff, error, True
        else:
            return joint_diff, error, False

    def _loop_angle_pub_cmd(self, cmd, joint_vals, speed=0.6, r_arm=True):
        success = False
        cmd(joint_vals, speed=speed, r_arm=r_arm)

        # wait 10s in worse case
        for i in range(int(10 / self.configs.ARM.WAIT_MIN_TIME)):
            res = self._angle_error_is_small(joint_vals, r_arm=r_arm)
            joint_diff, error, eflag = res

            if eflag:
                success = True
                break

            rospy.sleep(self.configs.ARM.WAIT_MIN_TIME)

        return success

    def _setup_joint_pub(self):
        self.joint_pub = rospy.Publisher(
            self.configs.ARM.ROSTOPIC_SET_JOINT,
            JointAnglesWithSpeed,
            queue_size=10)

    def _callback_joint_states(self, msg):
        """
        ROS subscriber callback for the arm (right) joint state
        (position, velocity) and the extra arm (left) joint state

        :param msg: Contains message published in topic
        :type msg: sensor_msgs/JointState
        """
        self.joint_state_lock.acquire()
        for idx, name in enumerate(msg.name):
            if name in self.arm_joint_names:
                if idx < len(msg.position):
                    self._joint_angles[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._joint_velocities[name] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self._joint_efforts[name] = msg.effort[idx]
            elif name in self.extra_arm_joint_names:
                if idx < len(msg.position):
                    self._extra_joint_angles[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._extra_joint_velocities[name] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self._extra_joint_efforts[name] = msg.effort[idx]
        self.joint_state_lock.release()

    def _pub_joint_positions(self, positions, speed, r_arm=True):
        command_msg = JointAnglesWithSpeed()

        if r_arm:
            command_msg.joint_names = self.arm_joint_names
        else:
            command_msg.joint_names = self.extra_arm_joint_names

        command_msg.joint_angles = positions
        speed = max(speed, 0.0)
        speed = min(speed, 1.0)
        command_msg.speed = speed
        command_msg.relative = False
        self.joint_pub.publish(command_msg)

    def _pub_joint_velocities(self, velocities):
        raise NotImplementedError(
            'Velocity control for Pepper not yet implemented!')

    def _pub_joint_torques(self, torques):
        raise NotImplementedError(
            'Torque control for Pepper not yet implemented!')
