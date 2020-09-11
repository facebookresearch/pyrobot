from abc import ABC, abstractmethod 

class Algorithm:
	"""Abstract base class for algorithms.
	"""
	def __init__( self, configs, 
						world,
						ros_launch_manager = None, 
						robots={}, 
						sensors={}, 
						algorithms={}, 
						
	):

		self.configs = configs
		self.robots = robots
		self.sensors = sensors
		self.algorithms = algorithms
		self.world = world
		self.check_cfg()

	@abstractmethod
	def check_cfg(self):
		pass
	