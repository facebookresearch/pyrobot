from pyrobot.algorithms.frame_transform import FrameTransform

from tf import TransformListener
import tf

import rospy


class TFTransform(FrameTransform):
    """
    Implementation of tf-based FrameTransform algorithms.
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
        super(FrameTransform, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )

        self.tf_listener = TransformListener()

    def get_transform(self, source_frame, target_frame):
        try:
            time = rospy.get_rostime()
            self.tf_listener.waitForTransform(
                source_frame, target_frame, time, rospy.Duration(3)
            )
            (trans, quat) = self.tf_listener.lookupTransform(
                source_frame, target_frame, time
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            raise RuntimeError(
                "Cannot fetch the transform from"
                " {0:s} to {1:s}".format(source_frame, target_frame)
            )
        return trans, quat

    def check_cfg(self):
        pass
