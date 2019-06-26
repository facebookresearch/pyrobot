from pyrobot import Robot
from subprocess import Popen, PIPE, STDOUT

robot = Robot('husky',
		use_base=True,
		use_arm=False,
		use_camera=False,
		use_gripper=False)

# linear_velocity in m/s
linear_velocity = 1.0

# rotational_velocity in radian / s
rotational_velocity = 0.5

# execution_time in seconds
execution_time = 2

i = 0
#launch gazebo
p = Popen(['rosnode', 'cleanup'])
p.wait()

args = 'base:=husky use_rviz:=false use_sim:=true'
args = args.split()
husky_p = Popen(['roslaunch', 'husky_gazebo', 'husky_empty_world.launch'] + args)
test_cmds = ['test_make_robot.py test_base_velocity_control.py' ' --botname husky']
run_test(test_cmds, 'basics.html')
exit_gazebo(husky_p)

while (i<100):
# Command to execute motion
	robot.base.set_vel(fwd_speed=linear_velocity,
                   turn_speed=rotational_velocity,
                   exe_time=execution_time)
	print(robot.base.get_state('odom'))
