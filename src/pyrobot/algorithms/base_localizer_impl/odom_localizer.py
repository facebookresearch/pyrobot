from pyrobot.algorithms.base_localizer import BaseLocalizer

from pyrobot.algorithms.base_controller_impl.base_control_utils import wrap_theta

import rospy
import tf

import copy

from nav_msgs.msg import Odometry

import numpy as np



class OdomLocalizer(BaseLocalizer):

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

        self.state = XYTState()

        self.robot_label = list(self.robots.keys())[0]

        self.bot_base = self.robots[self.robot_label]["base"]

        self.subscriber = rospy.Subscriber(
            self.bot_base.configs["ROSTOPIC_ODOMETRY"],
            Odometry,
            lambda msg: self._odometry_callback(msg, "state"),
        )

    def get_odom_state(self):
        state = self.state.state_f
        return (np.array(state, dtype=np.float32).T).copy()

    def check_cfg(self):
        super().check_cfg()
        
        robot_label = list(self.robots.keys())[0]
        assert (
            "ROSTOPIC_ODOMETRY" in self.robots[robot_label]["base"].configs.keys()
        ), "ROSTOPIC_ODOMETRY required for odom localizers!"

    def _odometry_callback(self, msg, state_var):
        """Callback function to populate the state object."""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            orientation_list)
        state = copy.deepcopy(getattr(self, state_var))
        state.update(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        setattr(self, state_var, state)


class XYTState(object):
    """
    This class object which can be used used to hold the pose of the base of
    the robot i.e, (x,y, yaw)
    """

    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.theta = 0.
        self.old_theta = 0
        self._state_f = np.array([self.x, self.y, self.theta],
                                 dtype=np.float32).T
        self.update_called = False

    def update(self, x, y, theta):
        """Updates the state being stored by the object."""
        theta = wrap_theta(theta - self.old_theta) + self.old_theta
        self.old_theta = theta
        self.theta = theta
        self.x = x
        self.y = y
        self._state_f = np.array([self.x, self.y, self.theta],
                                 dtype=np.float32).T
        self.update_called = True

    @property
    def state_f(self):
        """Returns the current state as a numpy array."""
        assert (self.update_called), "Odometry callback hasn't been called."
        return self._state_f

