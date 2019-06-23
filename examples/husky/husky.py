import numpy as np
import rospy
from absl import flags, app
from pyrobot import Robot

FLAGS = flags.FLAGS
flags.DEFINE_string('base_controller', 'proportional', 'One of ilqr, proportional, movebase.')
flags.DEFINE_string('base_planner', 'none', 'movebase or none')
flags.DEFINE_string('botname', 'husky', 'Robot name, locobot, locobot_lite, husky ...')


def main(_):
	robot = Robot('husky',
 			  use_arm=False,
			  use_camera=False,
			  use_gripper=False,
			  base_config={'base_controller': FLAGS.base_controller,
						 'base_planner': FLAGS.base_planner})

	# linear_velocity in m/s
	linear_velocity = 1.0

	# rotational_velocity in radian / s
	rotational_velocity = 0.5

	# execution_time in seconds
	execution_time = 1
	i = 0
	while (i<100):
		# Command to execute motion
		robot.base.set_vel(fwd_speed=linear_velocity,
                   turn_speed=rotational_velocity,
                   exe_time=execution_time)
		print(robot.base.get_state('odom'))

	bot.base.stop()

if __name__ == '__main__':
    app.run(main)
