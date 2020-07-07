#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from pyrobot import Robot
from projector_system.srv import RobotMoving, RobotMovingResponse


import time
import sys



def odom_callback(data):
    pub = rospy.Publisher('proj_odom', Odometry, queue_size=10)
    rospy.init_node('odom_talker', anonymous=True)
    rate = rospy.Rate(10) 
    
    while not rospy.is_shutdown():
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

def odom_listener():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()


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
    
    odom_listener()

    #if sys.argv[0] == "countdown":
		#countdown_server()

    #elif sys.argv[0] == "arrow":
	#	try:
    #    	talker()
    # 	except rospy.ROSInterruptException:
    #    	pass




