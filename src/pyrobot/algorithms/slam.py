from .algorithm import Algorithm


class SLAM(Algorithm):
    """
    Base class of SLAM algorithms.

    Specifically, this algorithm handles updating map, getting base state in world frame.
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

    def get_pointcloud_map(self):
        """
        This function fetches the 3D point cloud reconstructed using sensory input along the trajectory.

        Returns:
            pts: points of the point cloud (shape: :math:`[N, 3]`)
            colors: color of the points (shape: :math:`[N, 3]`); None if using depth camera only
        """
        raise NotImplementedError()

    def get_occupancy_map(self):
        """
        This function fetches the occupancy map reconstructed using sensory input along the trajectory.

        Returns:
            occ_map: 2D occupancy map (shape: :math:`[(x_max - x_min) / resolution, (y_max - y_min) / resolution]`)
        """
        raise NotImplementedError()

    def check_cfg(self):
        """
        This function checks required configs shared by all SLAM instances in constructor.

        Specifically, it checks for:
            1) The SLAM algorithm handles one robot, and one robot only
            2) The robot has a base
            3) The robot has a camera

        For any algorithm specific config checks, please extend the check_cfg function
            with customized algorithm config checks after calling this function
            using `super().check_cfg()`
        """

        assert len(self.robots.keys()) == 1, "One SLAM only handle one base!"
        robot_label = list(self.robots.keys())[0]
        assert "base" in self.robots[robot_label].keys(), "base required for SLAM!"
        assert "camera" in self.robots[robot_label].keys(), "camera required for SLAM!"
        # TODO: check for resolution

    def get_class_name(self):
        return "SLAM"
