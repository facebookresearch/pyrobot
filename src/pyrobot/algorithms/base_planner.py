from .algorithm import Algorithm


class BasePlanner(Algorithm):
    """
    Base class of Base Planner algorithms.

    Specifically, this algorithm handles generating base trajectory and execute the trajectory.
    """

    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):
        super(Algorithm, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )

    def get_plan_absolute(self, x, y, theta):
        """
        This function takes in desired x, y, theta position in world frame,
            and generate a plan as a list of poses in the world frame

        Args:
            x: desired translation along x-axis in world frame. Unit: meters.
            y: desired translation along y-axis in world frame. Unit: meters.
            theta: desired orientation in world frame. Unit: radius.
        Returns:
            plan: list of x, y, t points along the planned trajectory (point1, point2, ..., pointn)
                each of the point would take form that
                point.pose.position.x, point.pose.position.y, point.pose.position.theta
                denotes x, y, t position of each point in world frame.

        If a plan cannot be generated, this function should raise an error.
        """
        raise NotImplementedError()

    def move_to_goal(self, x, y, theta):
        """
        This function takes in desired x, y, theta position in world frame,
            compute a plan as a list of poses in the world frame,
            and move the base to the desired pose following the plan

        Args:
            x: desired translation along x-axis in world frame. Unit: meters.
            y: desired translation along y-axis in world frame. Unit: meters.
            theta: desired orientation in world frame. Unit: radius.
        Returns:
            status: bool. True if the base gets to the target xyt pose; False otherwise.

        If a plan cannot be generated or the generated plan cannot be executed, this function should raise an error.
        """
        raise NotImplementedError()

    def check_cfg(self):
        """
        This function checks required configs shared by all base planner instances in constructor.

        Specifically, it checks for:
            1) The base planner algorithm handles one robot, and one robot only
            2) The robot has a base

        For any algorithm specific config checks, please extend the check_cfg function
            with customized algorithm config checks after calling this function
            using `super().check_cfg()`
        """

        assert len(self.robots.keys()) == 1, "One Planner only handle one base!"
        robot_label = list(self.robots.keys())[0]
        assert (
            "base" in self.robots[robot_label].keys()
        ), "base required for base planners!"

    def get_class_name(self):
        return "BasePlanner"
