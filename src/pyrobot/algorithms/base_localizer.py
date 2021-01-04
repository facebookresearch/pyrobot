from .algorithm import Algorithm


class BaseLocalizer(Algorithm):
    """
    Base class of Base Localizer algorithms.

    Specifically, this algorithm handles getting base state in world frame.
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

    def get_odom_state(self):
        """
        Returns:
            xyt_state: a size-3 array in the form of [[x], [y], [t]]
            where x, y is the 2D translation from world frame to robot base in meters
            and t is the planer orientation from world frame to robot base in radius.
        """
        raise NotImplementedError()

    def check_cfg(self):
        """
        This function checks required configs shared by all base localizer instances in constructor.

        Specifically, it checks for:
            1) The base localizer algorithm handles one robot, and one robot only
            2) The robot has a base

        For any algorithm specific config checks, please extend the check_cfg function
            with customized algorithm config checks after calling this function
            using `super().check_cfg()`
        """

        assert len(self.robots.keys()) == 1, "One Localizer only handle one base!"
        robot_label = list(self.robots.keys())[0]
        assert (
            "base" in self.robots[robot_label].keys()
        ), "base required for base localizers!"

    def get_class_name(self):
        return "BaseLocalizer"
