from pyrobot import Robot
import time

# base_config_dict is a dictionary that contains different base configuration
# parameters. 'base_controller' can be set to 'ilqr' or 'proportional' or
# 'movebase' to use the respective controllers.
base_config_dict={'base_controller': 'ilqr'} 

# crate the Robot object with the specified base_config
robot = Robot('locobot', base_config=base_config_dict)
.
print(robot.base)


# target position we want the base to go to
pos1 = [.3,.3,0] # this is a 2D pose of the form [x, y, yaw]

# Now command the robot to go to the target pose in the enviroment
# 'go_to_absolute' assumes that the target is in world frame.
robot.base.go_to_absolute(pos1) 


time.sleep(2) 
#robot.base.go_to_relative([0,0,1])
robot.base.go_to_absolute([0,0,0])


# Targets can be specified in robot's coordinate frame.
# robot.base.go_to_relative(target_position)
