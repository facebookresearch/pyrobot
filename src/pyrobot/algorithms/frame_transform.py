from .algorithm import Algorithm


class FrameTransform(Algorithm):
    """
    Base class of frame transformation algorithms.

    Specifically, this algorithm handles getting transformation of the latest known timestamp, 
    including position and orientation, between two frames.
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

    def get_transform(self, source_frame, target_frame):
        """
        This function takes in a source frame and a target frame,
            get a transform from source frame to target frame.

        Args:
            source_frame: string of source frame name
            target_frame: string of target frame name
        Returns:
            position: list of position vector in the form of [x, y, z]. Unit: meters.
            quarternion: list of quaternion vector in the form of [x, y, z, w]

        If a transform cannot be found, this function should raise an error.
        """
        raise NotImplementedError()

    def check_cfg(self):
        """
        This function checks required configs shared by all frameTransform instances in constructor.
        """
        pass

    def get_class_name(self):
        return "FrameTransform"
