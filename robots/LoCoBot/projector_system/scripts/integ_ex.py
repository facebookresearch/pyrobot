#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from pyrobot import Robot
import time
import numpy as np
from matplotlib import pyplot as plt
	



def main():
	robot_started = True
	velocity = 0.2
	distance = 1.8
	time_to_goal = distance/velocity
	init_time = int(time_to_goal)
	fig, ax = plt.subplots()

	

def callback(data):
	linear = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
	angular = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]
        
	if linear !=[0,0,0] or angular !=[0,0,0]:
		countdown()	


def listener():
	rospy.init_node('listener', anonymous=True)
    	rospy.Subscriber("odom", Odometry, callback)
    	rospy.spin()


def countdown():


if __name__ == '__main__':
    	main()
