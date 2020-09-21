from .algorithm import Algorithm


class BasePlanner(Algorithm):
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
        super(Algorithm, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )

    def get_plan_absolute(self, x, y, theta):
        raise NotImplementedError()

    def move_to_goal(self, goal):
        raise NotImplementedError()

    def check_cfg(self):
        raise NotImplementedError()

    def get_class_name(self):
        return "BasePlanner"
