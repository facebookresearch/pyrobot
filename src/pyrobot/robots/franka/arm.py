import numpy as np
import rospy

import enum
import rospy
import warnings
import quaternion
import numpy as np
from copy import deepcopy
from rospy_message_converter import message_converter

from franka_core_msgs.msg import JointCommand, RobotState, EndPointState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .utils import wait_for

from franka_tools import (
    FrankaFramesInterface,
    FrankaControllerManagerInterface,
    JointTrajectoryActionClient,
    CollisionBehaviourInterface,
)

from pyrobot.robots.ros_arm import ROSArm


class FrankaArm(ROSArm):
    """
    This class has functionality to control a robotic arm
    in joint and task space (with and
    without any motion planning), for position/velocity/torque control, etc.
    """

    def __init__(self, configs):
        super(FrankaArm, self).__init__(
            configs=configs,
        )

        self._ns = "/franka_ros_interface"

        self._errors = dict()

        self._command_msg = JointCommand()

        # neutral pose joint positions
        self._neutral_pose_joints = self.configs.NEUTRAL_POSE

        self._frames_interface = FrankaFramesInterface()
        try:
            self._collision_behaviour_interface = CollisionBehaviourInterface()
        except rospy.ROSException:
            rospy.loginfo(
                "Collision Service Not found. It will not be possible to change collision behaviour of robot!"
            )
            self._collision_behaviour_interface = None
        self._ctrl_manager = FrankaControllerManagerInterface(ns=self._ns, sim=False)

        self._speed_ratio = 0.15

        rospy.on_shutdown(self._clean_shutdown)

        self.set_joint_position_speed(self._speed_ratio)

    def go_home(self):
        """
        Reset robot to default home position
        """
        self.set_joint_positions(self._neutral_pose_joints)

    def set_joint_positions(self, positions, wait=True, **kwargs):
        """
        Sets the desired joint angles for all arm joints

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
        if wait:
            threshold = 0.005
            curr_controller = self._ctrl_manager.set_motion_controller(
                self._ctrl_manager.joint_trajectory_controller
            )

            min_traj_dur = 0.5
            traj_client = JointTrajectoryActionClient(joint_names=self.arm_joint_names)
            traj_client.clear()

            dur = np.maximum(
                np.abs(np.array(positions) - np.array(self.get_joint_angles()))
                / self.configs.JOINT_VELOCITY_LIMITS,
                min_traj_dur,
            )

            traj_client.add_point(positions=self.get_joint_angles(), time=0.0001)
            traj_client.add_point(
                positions=positions, time=max(dur) / self._speed_ratio
            )

            diffs = (np.array(positions) - np.array(self.get_joint_angles())).tolist()

            fail_msg = "FrankaArm: franka failed to reach commanded joint positions."

            traj_client.start()  # send the trajectory action request

            wait_for(
                test=lambda: traj_client.result() is not None
                or (all(diff < threshold for diff in diffs)),
                timeout=10,
                timeout_msg=fail_msg,
                rate=100,
                raise_on_error=False,
            )
            res = traj_client.result()
            if res is not None and res.error_code:
                rospy.loginfo("Trajectory Server Message: {}".format(res))

            rospy.sleep(0.5)

            self._ctrl_manager.set_motion_controller(curr_controller)

            rospy.loginfo("FrankaArm: position control complete")
        else:
            self._pub_joint_positions(positions)

    def set_joint_velocities(self, velocities, **kwargs):
        """
        Sets the desired joint velocities for all arm joints

        :param velocities: target joint velocities
        :type velocities: list
        """
        self._pub_joint_velocities(velocities)

    def set_joint_torques(self, torques, **kwargs):
        """
        Sets the desired joint torques for all arm joints

        :param torques: target joint torques
        :type torques: list
        """
        self._pub_joint_torques(torques)

    def _setup_joint_pub(self):
        self.joint_pub = rospy.Publisher(
            self.configs.ROSTOPIC_SET_JOINT, JointState, queue_size=1
        )

    def _pub_joint_positions(self, positions):
        self._command_msg.names = self.arm_joint_names
        self._command_msg.position = list(positions)
        self._command_msg.mode = JointCommand.POSITION_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def _pub_joint_velocities(self, velocities):
        self._command_msg.names = self.arm_joint_names
        self._command_msg.velocity = list(velocities)
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def _pub_joint_torques(self, torques):
        self._command_msg.names = self.arm_joint_names
        self._command_msg.effort = list(torques)
        self._command_msg.mode = JointCommand.TORQUE_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def _setup_joint_pub(self):
        queue_size = 1
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._joint_command_publisher = rospy.Publisher(
                "/franka_ros_interface/motion_controller/arm/joint_commands",
                JointCommand,
                tcp_nodelay=True,
                queue_size=queue_size,
            )

        self._pub_joint_cmd_timeout = rospy.Publisher(
            "/franka_ros_interface/motion_controller/arm/joint_command_timeout",
            Float64,
            latch=True,
            queue_size=10,
        )

    def _clean_shutdown(self):
        self._pub_joint_cmd_timeout.unregister()
        self._joint_command_publisher.unregister()

    def set_joint_position_speed(self, speed=0.3):
        """
        Set ratio of max joint speed to use during joint position
        moves (only for move_to_joint_positions).

        Set the proportion of maximum controllable velocity to use
        during joint position control execution. The default ratio
        is `0.3`, and can be set anywhere from [0.0-1.0] (clipped).
        Once set, a speed ratio will persist until a new execution
        speed is set.

        :type speed: float
        :param speed: ratio of maximum joint speed for execution
                      default= 0.3; range= [0.0-1.0]
        """
        if speed > 0.3:
            rospy.logwarn(
                "ArmInterface: Setting speed above 0.3 could be risky!! Be extremely careful."
            )

        self._speed_ratio = speed
