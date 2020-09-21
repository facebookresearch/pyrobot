from .algorithm import Algorithm


class BaseLocalizer(Algorithm):
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

    def get_odom_state(self):
        raise NotImplementedError()

    def check_cfg(self):
        raise NotImplementedError()

    def get_class_name(self):
        return "BaseLocalizer"
