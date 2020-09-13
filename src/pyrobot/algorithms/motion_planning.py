from .algorithm import Algorithm 

class MotionPlanner(Algorithm):
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
		super(Algorithm, self).__init__(
										configs, 
										world, 
										ros_launch_manager, 
										robots, 
										sensors, 
										algorithms, 
										)

	def plan_end_effector_pose(self, position, orientation):
		raise NotImplementedError()

	def plan_joint_angles(self, target_joint):
		raise NotImplementedError()

	def compute_cartesian_path(self, joint_pos, target_frame):
		raise NotImplementedError()

	def check_cfg(self):
		raise NotImplementedError()

	def get_class_name(self):
		return "MotionPlanner"