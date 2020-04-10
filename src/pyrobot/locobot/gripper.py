# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import threading

import numpy as np
import rospy
from pyrobot.core import Gripper
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_msgs.msg import Int8


class LoCoBotGripper(Gripper):
    """
    Interface for gripper
    """

    def __init__(self, configs, wait_time=3):
        """
        The constructor for LoCoBotGripper class.

        :param configs: configurations for gripper
        :param wait_time: waiting time for opening/closing gripper
        :type configs: YACS CfgNode
        :type wait_time: float
        """
        super(LoCoBotGripper, self).__init__(configs=configs)
        self._gripper_state_lock = threading.RLock()
        self._gripper_state = None
        # Publishers and subscribers
        self.wait_time = wait_time
        self.pub_gripper_close = rospy.Publisher(
            self.configs.GRIPPER.ROSTOPIC_GRIPPER_CLOSE, Empty, queue_size=1
        )
        self.pub_gripper_open = rospy.Publisher(
            self.configs.GRIPPER.ROSTOPIC_GRIPPER_OPEN, Empty, queue_size=1
        )
        rospy.Subscriber(
            self.configs.GRIPPER.ROSTOPIC_GRIPPER_STATE,
            Int8,
            self._callback_gripper_state,
        )

    def get_gripper_state(self):
        """
        Return the gripper state. 

        :return: state

                 state = -1: unknown gripper state

                 state = 0: gripper is fully open

                 state = 1: gripper is closing

                 state = 2: there is an object in the gripper

                 state = 3: gripper is fully closed

        :rtype: int
        """
        self._gripper_state_lock.acquire()
        g_state = self._gripper_state
        self._gripper_state_lock.release()
        return g_state

    def open(self, wait=True):
        """
        Commands gripper to open fully

        :param wait: True if blocking call and will return
                     after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.pub_gripper_open.publish()
        if wait:
            rospy.sleep(self.wait_time)

    def reset(self, wait=True):
        """
        Utility function to reset gripper if it is stuck

        :param wait: True if blocking call and will return
                     after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.open(wait)
        self.close(wait)
        self.open(wait)

    def close(self, wait=True):
        """
        Commands gripper to close fully

        :param wait: True if blocking call and will return
                     after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.pub_gripper_close.publish()
        if wait:
            rospy.sleep(self.wait_time)

    def _callback_gripper_state(self, msg):
        """
        ROS subscriber callback for gripper state

        :param msg: Contains message published in topic
        :type msg: std_msgs/Int8
        """
        self._gripper_state_lock.acquire()
        self._gripper_state = msg.data
        self._gripper_state_lock.release()
