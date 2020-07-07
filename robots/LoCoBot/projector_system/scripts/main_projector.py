#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from pyrobot import Robot
from projector_system.srv import RobotMoving, RobotMovingResponse


import time
import sys



#def callback(data):
#    linear = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
#    angular = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]
#	
#	# Countdown
#	if sys.argv[0] == "countdown":
#		if linear !=[0,0,0] or angular !=[0,0,0]:
#        	# Call circle_anim - or possibly create a publisher that tells circle_anim to start, as we've already launched it in the launch file
#			# but then circle_anim has a subscriber, which at that point why wouldn't we just subscribe to /odom? Runs into issues of the script constantly running like callback does here
#			execfile('circle_anim.py')

#	elif sys.argv[0] == "arrow":
#		# Create a publisher that publishes angular component of /odom and 

#def listener():
#	rospy.init_node('listener', anonymous=True)
#    rospy.Subscriber("odom", Odometry, callback)
#    rospy.spin()


def countdown_server():
    rospy.init_node('robot_moving_server')
    s = rospy.Service('robot_moving', RobotMoving, motion_check)
    print("Ready to check for robot motion.")
    rospy.spin()

def motion_check(req):
    
    linear = [0,0,0]
    angular = [0,0,0]

    while linear==[0,0,0] and angular==[0,0,0]:
        data = rospy.wait_for_message("/odom", Odometry)
        linear = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
        angular = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]


    if linear !=[0,0,0] or angular !=[0,0,0]:
		return RobotMovingResponse(True)


if __name__ == '__main__':
    countdown_server()

    #elif sys.argv[0] == "arrow":
