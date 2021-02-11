import os

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
########################
# Duck Type Dictionary #
########################
class CollectionStruct:
    def __init__(self, **entries):
        self.__dict__.update(entries)

    def __getitem__(self, key):
        return self.__dict__[key]

    def __setitem__(self, key, value):
        self.__dict__[key] = value

    def __repr__(self):
        return self.__dict__.__repr__()

    def keys(self):
        return self.__dict__.keys()

    def add_module(self, label, module):
        self.__dict__[label] = module

############
# Registry #
############
class GlobalRegistry:
    def __init__(self):
        self.spec = CollectionStruct()
        self.spec.add_module("world", [])
        self.spec.add_module("robot", [])
        self.spec.add_module("algorithm", [])
        self.spec.add_module("sensor", [])

    def available_envs(self):
        return self.spec.envs

    def available_robots(self):
        return self.spec.robots

    def available_sensors(self):
        return self.spec.sensors

    def available_algorithms(self):
        return self.spec.algorithms

    def available_modules(self):
        return self.spec

    def register(self, module_type, module_name):
        if os.path.isfile(os.path.join(ROOT_DIR, "hydra_config", module_type, "{}.yaml".format(module_name))):
            self.spec[module_type].append(module_name)
        else:
        	print(os.path.join(ROOT_DIR, "hydra_config", module_type, "{}.yaml".format(module_name)))

    def module_exist(self, module_name):
        if isinstance(module_name, str):
            for key in self.spec.keys():
                if module_name in self.spec[key]:
                    return key
        return None

registry = GlobalRegistry()

registry.register("world", "simple_env")
registry.register("world", "locobot_base_env")
registry.register("world", "locobot_arm_env")
registry.register("world", "franka_arm_env")

registry.register("algorithm", "default_camera_transform")
registry.register("algorithm", "franka_kdl_kinematics")
registry.register("algorithm", "franka_moveit_planner")
registry.register("algorithm", "locobot_gpmp_control")
registry.register("algorithm", "locobot_ilqr_control")
registry.register("algorithm", "locobot_kdl_kinematics")
registry.register("algorithm", "locobot_movebase_control")
registry.register("algorithm", "locobot_movebase_planner")
registry.register("algorithm", "locobot_moveit_kin_planner")
registry.register("algorithm", "locobot_moveit_planner")
registry.register("algorithm", "locobot_odom_localizer")
registry.register("algorithm", "locobot_proportional_control")
registry.register("algorithm", "locobot_vslam")
registry.register("algorithm", "locobot_vslam_localizer")
registry.register("algorithm", "tf_transform")