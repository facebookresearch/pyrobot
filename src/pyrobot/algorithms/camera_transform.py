from .algorithm import Algorithm


class CameraTransform(Algorithm):
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

    def pix_to_pt(self, rows, columns, depth_img, rgb_img=None, in_cam=False):
        raise NotImplementedError()

    def pcd_from_img(self, depth_img, rgb_img=None, in_cam=False):
        raise NotImplementedError()

    def get_transform_matrix(self, src, tgt):
        raise NotImplementedError()

    def get_intrinsics(self):
        raise NotImplementedError()

    def check_cfg(self):
        raise NotImplementedError()

    def get_class_name(self):
        return "CameraTransform"
