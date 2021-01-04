from .algorithm import Algorithm


class CameraTransform(Algorithm):
    """
    Base class of Camera Transform algorithms.

    Specifically, this algorithm handles getting transformation from image to 3D points.
    It also give access to camera intrinsics and extrinsics.
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

    def pix_to_pt(self, rows, columns, depth_img, rgb_img=None, in_cam=True):
        """
        This function takes in row and column indices of depth and rgb image,
            and map to 3d point in camera frame or in specified base frame.

        Args:
            rows: list-like vector of indices of the rows of queried pixels, length is number of queried pixels.
            columns: list-like vector of indices of the columns of queried pixels, length should be the same as `rows`.
            depth_img: array of depth image with shape (Camera.height, Camera.width)
            rgb_img: array of rgb image with shape (Camera.height, Camera.width, 3). Default: None.
                If None, the function would only return 3D points but not color.
            in_cam: bool, specify if the returned 3d points are in camera frame. Default: True.
                If True, the function would return 3D points w.r.t camera frame;
                else it would return points w.r.t base frame specified in camera configs.

        Returns:
            pts: array of queried pixel positions in 3D with shape (n, 3).
                Each of the n entries is an [x, y, z] vector
            colors: array of queried pixel colors in 3D with shape (n, 3).
                Each of the n entries is an [r, g, b] vector
                If rgb_img is None, colors would be None.
        """
        raise NotImplementedError()

    def pcd_from_img(self, depth_img, rgb_img=None, in_cam=True):
        """
        This function takes in depth and rgb image,
            and map to 3d point in camera frame or in specified base frame.

        Args:
            depth_img: array of depth image with shape (Camera.height, Camera.width)
            rgb_img: array of rgb image with shape (Camera.height, Camera.width, 3). Default: None.
                If None, the function would only return 3D points but not color.
            in_cam: bool, specify if the returned 3d point cloud are in camera frame. Default: True.
                If True, the function would return 3D point cloud w.r.t camera frame;
                else it would return points w.r.t base frame specified in camera configs.

        Returns:
            pts: array of queried pixel positions in 3D with shape (n, 3),
                where n is the number of valid points in depth image.
                Each of the n entries is an [x, y, z] vector
            colors: array of queried pixel colors in 3D with shape (n, 3).
                where n is the number of valid points in depth image.
                Each of the n entries is an [r, g, b] vector
                If rgb_img is None, colors would be None.
        """
        raise NotImplementedError()

    def get_transform_matrix(self, source_frame, target_frame):
        """
        This function takes in a source frame and a target frame,
            get a transform from source frame to target frame.

        Args:
            source_frame: string of source frame name
            target_frame: string of target frame name

        Returns:
            transformation_matrix: a 4x4 matrix structured as:
            M = [
                [Rxx, Rxy, Rxz, tx],
                [Ryx, Ryy, Ryz, ty],
                [Rzx, Rzy, Rzz, tz],
                [0,   0,   0,   1 ]
            ]
            such that for any point p = [x, y, z] in source frame,
            construct p'  = [x, y, z, 1], Mp' gives you rotated and translated point in target frame;
            construct p'' = [x, y, z, 0], Mp'' gives you rotated point (no translation) in target frame.
        """
        raise NotImplementedError()

    def get_intrinsics(self):
        """
        Returns:
            intrinsics_matrix: a 3x3 matrix structured as:
            K = [
                [fx, 0,  cx],
                [0,  fy, cy],
                [0,  0,  1 ],
            ]
            where f is focal length and c is center pixel.
        """
        raise NotImplementedError()

    def check_cfg(self):
        """
        This function checks required configs shared by all camera transform instances in constructor.

        Specifically, it checks for:
            1) The camera transform algorithm handles one camera, and one camera only
            2) The camera is either a component of a robot or a sensor
            3) The camera has a base frame and an camera frame
            4) The camera has intrinsic params: fx, fy, cx, cy
            5) The camera has image size specified

        For any algorithm specific config checks, please extend the check_cfg function
            with customized algorithm config checks after calling this function
            using `super().check_cfg()`
        """
        assert (
            len(self.robots.keys()) + len(self.sensors.keys()) == 1
        ), "One camera transform only handle one camera!"

        if len(self.robots.keys()) > 0:
            robot_label = list(self.robots.keys())[0]
            assert (
                "camera" in self.robots[robot_label].keys()
            ), "Camera required for CameraTransform!"
            camera = self.robots[robot_label]["camera"]
        else:
            sensor_label = list(self.sensors.keys())[0]
            camera = self.sensors[sensor_label]

        assert (
            "BASE_FRAME" in camera.configs.keys()
        ), "BASE_FRAME required for in_cam transformation!"

        assert (
            "RGB_CAMERA_CENTER_FRAME" in camera.configs.keys()
        ), "RGB_CAMERA_CENTER_FRAME required for camera intrinsics!"

        assert (
            "Camera.fx" in camera.configs.keys()
        ), "Camera.fx required for camera intrinsics!"

        assert (
            "Camera.fy" in camera.configs.keys()
        ), "Camera.fy required for camera intrinsics!"

        assert (
            "Camera.cx" in camera.configs.keys()
        ), "Camera.cx required for camera intrinsics!"

        assert (
            "Camera.cy" in camera.configs.keys()
        ), "Camera.cy required for camera intrinsics!"

        assert (
            "Camera.height" in camera.configs.keys()
        ), "Camera.height required for camera intrinsics!"

        assert (
            "Camera.width" in camera.configs.keys()
        ), "Camera.width required for camera intrinsics!"

    def get_class_name(self):
        return "CameraTransform"
