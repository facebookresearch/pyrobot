from .algorithm import Algorithm 

class Kinematics(Algorithm):
	"""base class of Kinematics algorithms. 
	Specifically, forward/inverse kinematics, and jacobian.
	"""
	def __init__(
		self,
		configs,
		world, 
		ros_launch_manager = None,
		robots={},
	):
		super(Algorithm, self).__init__(
										configs, 
										world, 
										ros_launch_manager, 
										robots, 
										{}, 
										{}, 
										)

	def inverse_kinematics(self, position, orientation):
		raise NotImplementedError()

	def forward_kinematics(self, joint_pos, target_frame):
		raise NotImplementedError()

	def check_cfg(self):
		raise NotImplementedError()