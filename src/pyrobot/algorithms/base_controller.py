from .algorithm import Algorithm


class BaseController(Algorithm):
    """
    Base class of Base controller algorithms.

    Specifically, this algorithm handles moving base to absolute in the world frame,
    relative location in the base frame, and track pre-generated trajectories.
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

    def go_to_relative(self, xyt_position, close_loop=True, smooth=True):
        """
        This function takes in desired xyt_position in base frame, and move the base to this xyt_position

        Args:
            xyt_position: a list-like object with size 3 in the form of [x, y, theta], 
                denoting desired translation along x-axis in base frame (meters), 
                         desired translation along y-axis in base frame (meters),
                         desired orientation in base frame (radius) respectively.
            close_loop: bool. Specify if the controller should work in close loop. Default: True
            smooth: bool. Specify if the controller should generate smooth motion. Default: True
        Returns:
            status: bool. True if the base gets to the target xyt pose; False otherwise.
        """
        raise NotImplementedError()

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=True):
        """
        This function takes in desired xyt_position in world frame, and move the base to this xyt_position

        Args:
            xyt_position: a list-like object with size 3 in the form of [x, y, theta], 
                denoting desired translation along x-axis in world frame (meters), 
                         desired translation along y-axis in world frame (meters),
                         desired orientation in world frame (radius) respectively.
            close_loop: bool. Specify if the controller should work in close loop. Default: True
            smooth: bool. Specify if the controller should generate smooth motion. Default: True
        Returns:
            status: bool. True if the base gets to the target xyt pose; False otherwise.
        """
        raise NotImplementedError()

    def track_trajectory(self, states, close_loop=True):
        """
        This function takes in a trajectory of (x, y, theta) points in world frame, 
        and move the base to follow the trajectory

        Args:
            states: a list of (x, y, theta) vectors [[x1, y1, theta1], ..., [xn, yn, thetan]] 
            such that in each entry
                x denotes desired translation along x-axis in world frame (meters), 
                y denotes desired translation along y-axis in world frame (meters),
                theta denotes desired orientation in world frame (radius) respectively.
            close_loop: bool. Specify if the controller should work in close loop. Default: True
        Returns:
            status: bool. True if the base gets to all xyt-points in `states`; False otherwise.
        """
        raise NotImplementedError()

    def stop(self):
        """
        This function stops the base.
        """
        raise NotImplementedError()

    def check_cfg(self):
        """
        This function checks required configs shared by all base controller instances in constructor.

        Specifically, it checks for:
            1) The base controller algorithm handles one robot, and one robot only
            2) The robot has a base
            3) The base has a control subscriber which allows the controller to publish velocity commands

        For any algorithm specific config checks, please extend the check_cfg function 
            with customized algorithm config checks after calling this function
            using `super().check_cfg()`
        """
        assert len(self.robots.keys()) == 1, "One Controller only handle one base!"
        robot_label = list(self.robots.keys())[0]
        assert (
            "base" in self.robots[robot_label].keys()
        ), "base required for base controllers!"

        bot_base = self.robots[robot_label]['base']
        assert (
            bot_base.ctrl_pub
        ), "control publisher required for base controllers!"

    def get_class_name(self):
        return "BaseController"
