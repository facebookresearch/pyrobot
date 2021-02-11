from abc import ABC, abstractmethod


class Algorithm(ABC):
    """Abstract base class for algorithms."""

    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):
        if "conf" not in configs.keys():
            self.configs = {}
        else:
            self.configs = configs.conf
        self.robots = robots
        self.sensors = sensors
        self.algorithms = algorithms
        self.world = world
        self.check_cfg()

        if "ros_launch" in configs.keys() and configs.ros_launch:
            if not ros_launch_manager:
                ros_launch_manager = world.ros_launch_manager
            ns = ""
            if "ns" in configs.keys() and configs.ns:
                ns = configs.ns
            ros_launch_manager.launch_cfg(configs.ros_launch, ns=ns)

    @abstractmethod
    def check_cfg(self):
        pass

    @abstractmethod
    def get_class_name(self):
        pass
