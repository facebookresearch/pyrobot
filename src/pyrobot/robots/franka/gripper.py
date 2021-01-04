import rospy
import actionlib
from copy import deepcopy
from sensor_msgs.msg import JointState

from .utils import wait_for

from franka_gripper.msg import (
    GraspAction,
    GraspGoal,
    HomingAction,
    HomingGoal,
    MoveAction,
    MoveGoal,
    StopAction,
    StopGoal,
    GraspEpsilon,
)

from pyrobot.robots.gripper import Gripper


class FrankaGripper(Gripper):
    def __init__(self, configs):

        self._joint_positions = dict()
        self._joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        self._joint_velocity = dict()
        self._joint_effort = dict()

        self._joint_states_state_sub = rospy.Subscriber(
            "/franka_gripper/joint_states",
            JointState,
            self._joint_states_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        self._exists = False

        # ----- Initial test to see if gripper is loaded
        try:
            rospy.get_param("/franka_gripper/robot_ip")
        except KeyError:
            rospy.loginfo("FrankaGripper: could not detect gripper.")
            return
        except (socket.error, socket.gaierror):
            print(
                "Failed to connect to the ROS parameter server!\n"
                "Please check to make sure your ROS networking is "
                "properly configured:\n"
            )
            sys.exit()

        # ----- Wait for the gripper device status to be true
        if not wait_for(
            lambda: len(self._joint_positions.keys()) > 0,
            timeout=2.0,
            timeout_msg=(
                "FrankaGripper: Failed to get gripper joint positions. Assuming no gripper attached to robot."
            ),
            raise_on_error=False,
        ):
            return
        self._exists = True

        self._gripper_speed = 0.05

        self._grasp_action_client = actionlib.SimpleActionClient(
            "/franka_gripper/grasp", GraspAction
        )

        self._move_action_client = actionlib.SimpleActionClient(
            "/franka_gripper/move", MoveAction
        )

        self._stop_action_client = actionlib.SimpleActionClient(
            "/franka_gripper/stop", StopAction
        )

        rospy.loginfo("GripperInterface: Waiting for gripper action servers... ")
        self._grasp_action_client.wait_for_server()
        self._move_action_client.wait_for_server()
        self._stop_action_client.wait_for_server()
        rospy.loginfo("GripperInterface: Gripper action servers found! ")

        self.MIN_FORCE = 0.01
        self.MAX_FORCE = 50  # documentation says upto 70N is possible as continuous force (max upto 140N)

        self.MIN_WIDTH = 0.0001
        self.MAX_WIDTH = 0.2

    def open(self):
        """
        Open gripper to max possible width.

        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        self._caller = "open gripper"
        self.move_joints(0.2)

    def close(self):
        """
        close gripper to till collision is detected.
        Note: This is not exactly doing what it should. The behaviour is
        faked by catching the error thrown when trying to grasp a very small
        object with a very small force. Since the gripper will actually hit the
        object before it reaches the commanded width, we catch the feedback
        and send the gripper stop command to stop it where it is.

        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        self._caller = "close gripper"

        def cb(_, result):
            if not result.success:
                width = max(
                    abs(
                        self._joint_positions["panda_finger_joint1"]
                        - self._joint_positions["panda_finger_joint2"]
                    ),
                    0.001,
                )
                self.grasp(width, 10)

        return self.grasp(0.001, 0.1, cb=cb)

    def move_joints(self, width, wait=True):
        """
        Moves the gripper fingers to a specified width.

        :param width: Intended opening width. [m]
        :param speed: Closing speed. [m/s]
        :param wait_for_result: if True, this method will block till response is
                                    recieved from server

        :type width: float
        :type speed: float
        :type wait_for_result: bool

        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        self._caller = "move_joints"

        goal = MoveGoal()

        goal.width = width
        goal.speed = self._gripper_speed

        self._move_action_client.send_goal(
            goal,
            done_cb=self._done_cb,
            active_cb=self._active_cb,
            feedback_cb=self._feedback_cb,
        )

    def grasp(
        self, width, force, epsilon_inner=0.005, epsilon_outer=0.005, wait=True, cb=None
    ):
        """
        Grasps an object.

        An object is considered grasped if the distance $d$ between the gripper fingers satisfies
        $(\text{width} - \text{epsilon_inner}) < d < (\text{width} + \text{epsilon_outer})$.

        :param width: Size of the object to grasp. [m]
        :param speed: Closing speed. [m/s]
        :param force: Grasping force. [N]
        :param epsilon_inner: Maximum tolerated deviation when the actual grasped width is smaller
                                than the commanded grasp width.
        :param epsilon_outer: Maximum tolerated deviation when the actual grasped width is wider
                                than the commanded grasp width.
        :param cb: Optional callback function to use when the service call is done

        :type width: float
        :type speed: float
        :type force: float
        :type epsilon_inner: float
        :type epsilon_outer: float

        :return: True if an object has been grasped, false otherwise.
        :rtype: bool
        """
        self._caller = "grasp_action"

        goal = GraspGoal()
        goal.width = width
        goal.speed = self._gripper_speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner=epsilon_inner, outer=epsilon_outer)
        if not cb:
            cb = self._done_cb

        self._grasp_action_client.send_goal(
            goal, done_cb=cb, active_cb=self._active_cb, feedback_cb=self._feedback_cb
        )

    def stop(self):
        """
        Stops a currently running gripper move or grasp.

        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        self._caller = "stop_action"

        goal = StopGoal()

        self._stop_action_client.send_goal(
            goal,
            done_cb=self._done_cb,
            active_cb=self._active_cb,
            feedback_cb=self._feedback_cb,
        )

    def _active_cb(self):
        rospy.logdebug("GripperInterface: '{}' request active.".format(self._caller))

    def _feedback_cb(self, msg):
        rospy.logdebug(
            "GripperInterface: '{}' request feedback: \n\t{}".format(self._caller, msg)
        )

    def _done_cb(self, status, result):
        rospy.logdebug(
            "GripperInterface: '{}' complete. Result: \n\t{}".format(
                self._caller, result
            )
        )

    def _joint_states_callback(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_positions[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]
