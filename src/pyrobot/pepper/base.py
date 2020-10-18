#!/usr/bin/env python
# coding: utf-8

import time
import rospy
import threading
import tf.transformations
from pyrobot.core import Base
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


class PepperBase(Base):
    """
    This is a base class for the Pepper robot
    """

    def __init__(self, configs):
        """
        Constructor for the PepperBase class.

        :param configs: configurations read from config file
        """
        super(PepperBase, self).__init__(configs=configs)
        self.is_moving = False
        self.odometry_state = [0.0, 0.0, 0.0]
        self.odometry_lock = threading.Lock()

        self.pose_ctrl_pub = rospy.Publisher(
            configs.BASE.ROSTOPIC_BASE_POS_COMMAND,
            PoseStamped,
            queue_size=1)

        self.odom_sub = rospy.Subscriber(
            configs.BASE.ROSTOPIC_ODOMETRY,
            Odometry,
            self._odometry_callback)

    def stop(self):
        """
        Stop the base
        """
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        self.ctrl_pub.publish(msg)

    def set_vel(self, speed_x, speed_y, speed_theta, exe_time=1):
        """
        Set the moving velocity of the base (relative to the robot's initial
        pose)

        :param speed_x: linear speed along the x axis of the robot
        :param speed_y: linear speed along the y axis of the robot
        :param speed_theta: angular speed along the z axis of the robot
        :param exe_time: execution time
        """
        speed_x = min(speed_x, self.configs.BASE.MAX_ABS_FWD_SPEED)
        speed_x = max(speed_x, -self.configs.BASE.MAX_ABS_FWD_SPEED)
        speed_y = min(speed_y, self.configs.BASE.MAX_ABS_FWD_SPEED)
        speed_y = max(speed_y, -self.configs.BASE.MAX_ABS_FWD_SPEED)
        speed_theta = min(speed_theta, self.configs.BASE.MAX_ABS_TURN_SPEED)
        speed_theta = max(speed_theta, -self.configs.BASE.MAX_ABS_TURN_SPEED)

        msg = Twist()
        msg.linear.x = speed_x
        msg.linear.y = speed_y
        msg.angular.z = speed_theta

        start_time = rospy.get_time()
        self.ctrl_pub.publish(msg)

        while rospy.get_time() - start_time < exe_time:
            rospy.sleep(1. / self.configs.BASE.BASE_CONTROL_RATE)

        self.stop()

    def go_to_relative(self, xyt_position, blocking=True, timeout=20.0):
        """
        Moves the robot to the given goal in the robot frame.

        :param xyt_position: The relative goal state of the form (x,y,t)
        :param blocking: If True the method will be a blocking call
        :param timeout: If the method is blocking, the duration of the movement
                        cannot exceed the timeout

        :type xyt_position: list
        :type blocking: bool
        :type timeout: float

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        msg = self._generate_pose_stamped(xyt_position, "base_footprint")
        return self._go_to_pose_stamped(
            msg,
            blocking=blocking,
            timeout=timeout)

    def go_to_absolute(self, xyt_position, blocking=True, timeout=20.0):
        """
        Moves the robot to the given goal state in the world frame.

        :param xyt_position: The goal state of the form (x,y,t)
                             in the world (map) frame.
        :param blocking: If True the method will be a blocking call
        :param timeout: If the method is blocking, the duration of the movement
                        cannot exceed the timeout

        :type xyt_position: list
        :type blocking: bool
        :type timeout: float

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        msg = self._generate_pose_stamped(xyt_position, "odom")
        return self._go_to_pose_stamped(
            msg,
            blocking=blocking,
            timeout=timeout)

    def get_state(self):
        """
        Returns the requested base pose in the (x,y,yaw) format.

        :return: pose: pose of the form [x, y, yaw]
        :rtype: list
        """
        with self.odometry_lock:
            return self.odometry_state

    def _generate_pose_stamped(self, xyt_position, frame_id):
        """
        Generates a PoseStamped message, given xyt_position and a frame_id
        frame

        :param xyt_position: The goal state of the form (x,y,t)
                             in the world (map) frame.
        :param frame_id: The name of the pose reference frame

        :type xyt_position: list
        :type frame_id: str

        :return: The PoseStamped message
        :rtype: PoseStamped
        """
        quaternions = tf.transformations.quaternion_from_euler(
            0.0,
            0.0,
            xyt_position[2])

        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose.position.x = xyt_position[0]
        msg.pose.position.y = xyt_position[1]
        msg.pose.orientation.x = quaternions[0]
        msg.pose.orientation.y = quaternions[1]
        msg.pose.orientation.z = quaternions[2]
        msg.pose.orientation.w = quaternions[3]

        return msg

    def _go_to_pose_stamped(self, msg, blocking=True, timeout=20.0):
        """
        Publishes a PoseStamped message on
        configs.BASE.ROSTOPIC_BASE_POS_COMMAND, and waits for the base to reach
        its destination if the method is a blocking call.

        :param msg: The PoseStamped to reach
        :param blocking: If True the method will be a blocking call
        :param timeout: If the method is blocking, the duration of the movement
                        cannot exceed the timeout

        :type msg: PoseStamped
        :type blocking: bool
        :type timeout: float

        :return: True if successful; False otherwise (timeout, etc.)
        :rtype: bool
        """
        self.pose_ctrl_pub.publish(msg)

        if blocking:
            start_time = time.time()
            while time.time() - start_time < 1.5:
                with self.odometry_lock:
                    if self.is_moving:
                        break

                rospy.sleep(1.0 / self.configs.BASE.BASE_CONTROL_RATE)

            start_time = time.time()
            while time.time() - start_time < timeout:
                with self.odometry_lock:
                    if not self.is_moving:
                        return True

                rospy.sleep(1.0 / self.configs.BASE.BASE_CONTROL_RATE)

            return False
        else:
            return True

    def _odometry_callback(self, msg):
        """
        Odometry callback, triggered when an Odometry message is published on
        configs.BASE.ROSTOPIC_ODOMETRY

        :param msg: The Odometry message

        :type msg: Odometry
        """
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w]

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            orientation_list)

        with self.odometry_lock:
            self.odometry_state = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw]

            try:
                assert msg.twist.twist.linear.x == 0.0
                assert msg.twist.twist.linear.y == 0.0
                assert msg.twist.twist.angular.z <= 1e-5

                self.is_moving = False
            except AssertionError:
                self.is_moving = True
