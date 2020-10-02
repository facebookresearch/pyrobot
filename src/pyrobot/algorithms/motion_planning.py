from .algorithm import Algorithm


class MotionPlanner(Algorithm):
    """
    Base class of Motion Planning algorithms.
    Specifically, this algorithm handles computing and executing of motion plans.

    The algorithm should take a robot model urdf, specified in a tree of joint frames, 
    and an end-effector frame, in the constructor.
    Motion plans should be computed w.r.t that urdf and end-effector frame.
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

    def plan_end_effector_pose(self, position, orientation):
        """
        This function takes in position and orientation of a desired end effector pose,
            generate a motion plan to the desired pose, and execute the plan.

        Args:
            position: position vector as list-like objects in the form of [x, y, z]. Unit: meters.
            orientation: orientation vector or matrix with one of the following forms:
                1) euler angles, as list-like object of form [x, y, z]
                2) quaternion, as list-like object of form [x, y, z, w]
                3) a 3x3 3D rotation matrix
        Returns:
            plan: list of timestamped joint angle points along the planned trajectory. Unit: radius.
                i.e. for a 5DoF arm with (joint1, joint2, ..., joint5) in urdf,
                this function should return a list of points 
                with point.positions, point.velocities and point.accelarations as vectors of size 5
                (angle1, angle2, ..., angle5)

        If a plan cannot be generated or the generated plan cannot be executed, this function should raise an error.
        """
        raise NotImplementedError()

    def plan_joint_angles(self, target_joint):
        """
        This function takes in desired joint position,
            generate a motion plan to the desired pose, and execute the plan.

        Args:
            target_joint: list-like object of joint angles correspond to the queried end effector, Unit: radius.
                for a 5DoF arm with (joint1, joint2, ..., joint5) in urdf,
                this function should return a vector with size 5
                of (angle1, angle2, ..., angle5)
        Returns:
            plan: list of timestamped joint angle points along the planned trajectory. Unit: radius.
                i.e. for a 5DoF arm with (joint1, joint2, ..., joint5) in urdf,
                this function should return a list of points 
                with point.positions, point.velocities and point.accelarations as vectors of size 5
                (angle1, angle2, ..., angle5)

        If a plan cannot be generated or the generated plan cannot be executed, this function should raise an error.
        """
        raise NotImplementedError()

    def compute_cartesian_path(self, joint_pos, target_frame):
        raise NotImplementedError()

    def check_cfg(self):
        """
        This function checks required configs shared by all motion planning instances in constructor.

        Specifically, it checks for:
            1) The motion planning algorithm handles one robot, and one robot only
            2) The robot has an arm
            3) The arm has a base frame, an end-effector frame, and urdf description.

        For any algorithm specific config checks, please extend the check_cfg function 
            with customized algorithm config checks after calling this function
            using `super().check_cfg()`
        """
        assert len(self.robots.keys()) == 1, "One motion planner only handle one arm!"
        robot_label = list(self.robots.keys())[0]
        assert (
            "arm" in self.robots[robot_label].keys()
        ), "Arm required for MotionPlanners!"
        assert (
            "ARM_BASE_FRAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "ARM_BASE_FRAME required for MotionPlanner!"
        assert (
            "EE_FRAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "EE_FRAME required for MotionPlanner!"
        assert (
            "ARM_ROBOT_DSP_PARAM_NAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "ARM_ROBOT_DSP_PARAM_NAME required for MotionPlanner!"

    def get_class_name(self):
        return "MotionPlanner"
