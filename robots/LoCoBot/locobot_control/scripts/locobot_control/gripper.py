#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Wrapper for robot gripper
@amurali
"""
import rospy
from std_msgs.msg import Empty

ROSTOPIC_GRIPPER_OPEN = "/gripper/open"
ROSTOPIC_GRIPPER_CLOSE = "/gripper/close"
ROSTOPIC_GRIPPER_STATE = "/gripper/state"


class Gripper(object):
    """
    Bare bones interface for gripper
    """

    def __init__(self):

        # Publishers and subscribers
        self.pub_gripper_close = rospy.Publisher(
            ROSTOPIC_GRIPPER_CLOSE, Empty, queue_size=1
        )
        self.pub_gripper_open = rospy.Publisher(
            ROSTOPIC_GRIPPER_OPEN, Empty, queue_size=1
        )

    def gripper_state_callback(self, msg):
        """
        ROS subscriber callback for gripper state (open, close, faulty, etc.)

        :param data: Contains message published in topic
        """
        self.state = msg.data

    def open(self, wait=True):
        """
        Commands gripper to open fully

        :param wait: True if blocking call and will return after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.pub_gripper_open.publish()
        if wait:
            rospy.sleep(4)

    def reset(self, wait=True):
        """
        Utility function to reset gripper if it is stuck

        :param wait: True if blocking call and will return after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.open(wait)
        self.close(wait)
        self.open(wait)

    def close(self, wait=True):
        """
        Commands gripper to close fully

        :param wait: True if blocking call and will return after target_joint is achieved, False otherwise
        :type wait: bool
        """
        self.pub_gripper_close.publish()
        if wait:
            rospy.sleep(4)


if __name__ == "__main__":
    rospy.init_node("gripper_node", anonymous=True)
    gripper = Gripper()
