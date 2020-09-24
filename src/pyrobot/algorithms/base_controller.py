from .algorithm import Algorithm


class BaseController(Algorithm):
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

    def go_to_relative(self, xyt_position, close_loop=True, smooth=True):
        raise NotImplementedError()

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=True):
        raise NotImplementedError()

    def track_trajectory(self, states, close_loop=True):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

    def check_cfg(self):
        raise NotImplementedError()

    def get_class_name(self):
        return "BaseController"
