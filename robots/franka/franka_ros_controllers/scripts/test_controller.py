import rospy
from franka_interface import ArmInterface
import numpy as np
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
from copy import deepcopy

names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

if __name__ == '__main__':
    

    rospy.init_node("test_node")
    r = ArmInterface()

    rate = rospy.Rate(400)

    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)

    r.move_to_neutral() # move to neutral pose before beginning

    initial_pose = deepcopy(r.joint_ordered_angles())

    raw_input("Hit Enter to Start")
    print "commanding"
    vals = deepcopy(initial_pose)
    count = 0


    while not rospy.is_shutdown():

        elapsed_time_ += period

        delta = 3.14 / 16.0 * (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2

        for j, _ in enumerate(vals):
            if j == 4:
                vals[j] = initial_pose[j] - delta
            else:
                vals[j] = initial_pose[j] + delta

        if count%500 == 0:
            print vals, delta
            print "\n ----  \n"
            print " "


        # r.set_joint_positions_velocities(vals, [0.0 for _ in range(7)]) # for impedance control
        r.set_joint_positions(dict(zip(r.joint_names(), vals))) # try this for position control 

        count += 1
        rate.sleep()