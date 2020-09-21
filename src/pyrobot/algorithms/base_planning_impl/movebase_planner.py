from pyrobot.algorithms.base_planner import BasePlanner
from pyrobot.robots.locobot.base_control_utils import build_pose_msg

import tf
import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

class MovebasePlanner(BasePlanner):
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
        super(BasePlanner, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )

        self.robot_label = list(self.robots.keys())[0]

        self.bot_base = self.robots[self.robot_label]["base"]

        self.MAP_FRAME = self.configs.MAP_FRAME
        self.BASE_FRAME = self.configs.BASE_FRAME
        self.point_idx = self.configs.TRACKED_POINT

        rospy.wait_for_service(self.configs.PLAN_TOPIC, timeout=3)
        try:
            self.plan_srv = rospy.ServiceProxy(self.configs.PLAN_TOPIC, GetPlan)
        except rospy.ServiceException:
            rospy.logerr(
                "Timed out waiting for the planning service. \
					Make sure build_map in script and \
						use_map in roslauch are set to the same value"
            )
        self.start_state = _build_pose_msg(0, 0, 0, self.BASE_FRAME)
        self.tolerance = self.configs.PLAN_TOL
        self._transform_listener = tf.TransformListener()

    def get_plan_absolute(self, x, y, theta):
        """
        Gets plan as a list of poses in the world
        frame for the given (x, y, theta
        """
        goal_state = build_pose_msg(x, y, theta, self.MAP_FRAME)
        start_state = self._transform_listener.transformPose(
            self.MAP_FRAME, self.start_state
        )
        response = self.plan_srv(
            start=start_state, goal=goal_state, tolerance=self.tolerance
        )

        status = True
        if len(response.plan.poses) == 0:
            status = False
        return response.plan.poses, status

    def move_to_goal(self, goal):
        """
        Moves the robot to the robot to given goal state in the absolute frame
        (world frame).

        :param goal: The goal state of the form (x,y,t) in
                     the world (map) frame.
        :param go_to_relative: This is a method that moves the robot the
                               appropiate relative goal while NOT taking map
                               into account.
        :type goal: list
        :type go_to_rative: function of the form foo([x,y,theta])
        """
        plan, plan_status = self.get_plan_absolute(goal[0], goal[1], goal[2])
        if not plan_status:
            rospy.loginfo("Failed to find a valid plan!")
            return False

        if len(plan) < self.point_idx:
            point = goal
        else:
            point = [
                plan[self.point_idx - 1].pose.position.x,
                plan[self.point_idx - 1].pose.position.y,
                0,
            ]

        g_angle, g_distance = self._compute_relative_ang_dist(goal)

        while g_distance > self.configs.TRESHOLD_LIN:

            plan, plan_status = self.get_plan_absolute(goal[0], goal[1], goal[2])

            if self._as.is_preempt_requested():
                return False

            if not plan_status:
                rospy.loginfo("Failed to find a valid plan!")
                return False

            if len(plan) < self.point_idx:
                point = goal
            else:
                point = [
                    plan[self.point_idx - 1].pose.position.x,
                    plan[self.point_idx - 1].pose.position.y,
                    0,
                ]

            angle, distance = self._compute_relative_ang_dist(point)
            g_angle, g_distance = self._compute_relative_ang_dist(goal)
            if not self.algorithms["BaseController"].go_to_relative([0, 0, angle]):
                return False
            if not self.algorithms["BaseController"].go_to_relative([distance, 0, 0]):
                return False

        g_angle, g_distance = self._compute_relative_ang_dist(goal)

        rospy.loginfo("Adujusting Orientation at goal..{}.".format(g_angle))

        pose_stamped = build_pose_msg(goal[0], goal[1], goal[2], self.MAP_FRAME)
        base_pose = self._transform_listener.transformPose(
            self.BASE_FRAME, pose_stamped
        )

        pose_quat = [
            base_pose.pose.orientation.x,
            base_pose.pose.orientation.y,
            base_pose.pose.orientation.z,
            base_pose.pose.orientation.w,
        ]
        euler = tf.transformations.euler_from_quaternion(pose_quat)
        if not self.algorithms["BaseController"].go_to_relative([0, 0, euler[2]]):
            return False

    def check_cfg(self):
        raise NotImplementedError()

    def _compute_relative_ang_dist(self, point2):
        # point 1 is the base point 2 is the point on the path
        # convert point 2 to base frame
        pose_stamped = build_pose_msg(point2[0], point2[1], 0, self.MAP_FRAME)
        base_pose = self._transform_listener.transformPose(
            self.BASE_FRAME, pose_stamped
        )

        y = base_pose.pose.position.y
        x = base_pose.pose.position.x

        angle = np.arctan2(y, x)
        distance = np.sqrt(y ** 2 + x ** 2)

        return (angle, distance)
