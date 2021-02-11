from .algorithm import Algorithm


class Kinematics(Algorithm):
    """
    Base class of Kinematics algorithms.
    Specifically, this algorithm handles computing of forward/inverse kinematics, and jacobian.

    The algorithm should take a robot model urdf, specified in a tree of joint frames, in the constructor,
    and compute forward/inverse kinematics and jacobian correspond to that urdf.
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

    def inverse_kinematics(self, position, orientation):
        """
        This function takes in position and orientation of end effector, and map to joint angles.

        Args:
            position: position vector as list-like objects in the form of [x, y, z]. Unit: meters.
            orientation: orientation vector or matrix with one of the following forms:
                1) euler angles, as list-like object of form [x, y, z]
                2) quaternion, as list-like object of form [x, y, z, w]
                3) a 3x3 3D rotation matrix
        Returns:
            joint_angles: list of joint angles correspond to the queried end effector. Unit: radius.
                i.e. for a 5DoF arm with (joint1, joint2, ..., joint5) in urdf,
                this function should return a vector with size 5
                of (angle1, angle2, ..., angle5)

        If an inverse kinematics solution cannot be found, this function should raise an error.
        """
        raise NotImplementedError()

    def forward_kinematics(self, joint_pos, target_frame):
        """
        This function takes in a list of joint positions and name of target frame, and map to transformation
            of the target frame w.r.t base frame.

        Args:
            joint_pos: list-like object of joint angles correspond to the queried end effector, Unit: radius.
                for a 5DoF arm with (joint1, joint2, ..., joint5) in urdf,
                this function should return a vector with size 5
                of (angle1, angle2, ..., angle5)
            target_frame: string of target frame name
        Returns:
            position: list of position vector in the form of [x, y, z]. Unit: meters.
            quarternion: list of quaternion vector in the form of [x, y, z, w]

        If a forward kinematics solution cannot be found, this function should raise an error.
        """
        raise NotImplementedError()

    def check_cfg(self):
        """
        This function checks required configs shared by all kinematics instances in constructor.

        Specifically, it checks for:
            1) The kinematic algorithm handles one robot, and one robot only
            2) The robot has an arm
            3) The arm has a base frame, an end-effector frame, and urdf description.

        For any algorithm specific config checks, please extend the check_cfg function
            with customized algorithm config checks after calling this function
            using `super().check_cfg()`
        """

        assert (
            len(self.robots.keys()) == 1
        ), "One Kinematics solver only handle one arm!"
        robot_label = list(self.robots.keys())[0]
        assert "arm" in self.robots[robot_label].keys(), "Arm required for kinematics!"
        assert (
            "ARM_BASE_FRAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "ARM_BASE_FRAME required for kinematics!"
        assert (
            "EE_FRAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "EE_FRAME required for kinematics!"
        assert (
            "ARM_ROBOT_DSP_PARAM_NAME" in self.robots[robot_label]["arm"].configs.keys()
        ), "ARM_ROBOT_DSP_PARAM_NAME required for kinematics!"

    def get_class_name(self):
        return "Kinematics"
