from pyrobot.algorithms.base_localizer import BaseLocalizer

import rospy
import tf

import copy

from nav_msgs.msg import Odometry

import numpy as np


def _wrap_theta(theta):
    return np.mod(theta + np.pi, np.pi * 2) - np.pi


class OdomLocalizer(BaseLocalizer):
    """base class of camera transformation algorithms."""

    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):
        super(BaseLocalizer, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )
        self.configs = configs
        self.state = [0.0, 0.0, 0.0]

        self.robot_label = list(self.robots.keys())[0]

        self.bot_base = self.robots[self.robot_label]["base"]

        self.subscriber = rospy.Subscriber(
            self.bot_base.configs["ROSTOPIC_ODOMETRY"],
            Odometry,
            lambda msg: self._odometry_callback(msg, "state"),
        )

    def get_odom_state(self):
        return (np.array(self.state, dtype=np.float32).T).copy()

    def check_cfg(self):
        assert len(self.robots.keys()) == 1, "One Localizer only handle one base!"
        robot_label = list(self.robots.keys())[0]
        assert (
            "base" in self.robots[robot_label].keys()
        ), "base required for base localizers!"
        assert (
            "ROSTOPIC_ODOMETRY" in self.robots[robot_label]["base"].config.keys()
        ), "ROSTOPIC_ODOMETRY required for odom localizers!"

    def _odometry_callback(self, msg, state_var):
        """Callback function to populate the state object."""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        state = copy.deepcopy(getattr(self, state_var))
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = _wrap_theta(yaw - self.state[2]) + self.state[2]
        setattr(self, state_var, state)
