from pyrobot.algorithms.frame_transform import FrameTransform 

from tf import TransformListener

import rospy

class TFTransform(FrameTransform):
	"""base class of Motion Planning algorithms. 
	Specifically, forward/inverse kinematics, and jacobian.
	"""
	def __init__(
		self,
		configs,
		world, 
		ros_launch_manager = None,
		robots={},
		sensors={},
		algorithms={}
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

	def get_transform(self, target_frame, source_frame):
		try:
			self.tf_listener.waitForTransform(
				target_frame, source_frame, rospy.Time(0), rospy.Duration(3)
			)
			(trans, quat) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			raise RuntimeError(
				"Cannot fetch the transform from"
				" {0:s} to {1:s}".format(target_frame, source_frame)
			)
		return trans, quat

	def check_cfg(self):
		pass
