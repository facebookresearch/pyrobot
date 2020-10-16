import rospy
import numpy as np
from franka_interface import ArmInterface

if __name__ == '__main__':
    rospy.init_node("test_robot")
    r = ArmInterface() # create arm interface instance (see https://justagist.github.io/franka_ros_interface/DOC.html#arminterface for all available methods for ArmInterface() object)
    cm = r.get_controller_manager() # get controller manager instance associated with the robot (not required in most cases)
    mvt = r.get_movegroup_interface() # get the moveit interface for planning and executing trajectories using moveit planners (see https://justagist.github.io/franka_ros_interface/DOC.html#franka_moveit.PandaMoveGroupInterface for documentation)

    rospy.loginfo("Commanding...\n")
    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)

    initial_pose = r.joint_angles() # get current joint angles of the robot

    jac = r.zero_jacobian() # get end-effector jacobian

    count = 0
    rate = rospy.Rate(1000)

    vals = r.joint_angles()
    while not rospy.is_shutdown():

        elapsed_time_ += period

        delta = 3.14 / 16.0 * (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2

        for j in r.joint_names():
            if j == r.joint_names()[4]:
                vals[j] = initial_pose[j] - delta
            else:
                vals[j] = initial_pose[j] + delta

        r.set_joint_positions(vals) # set joint positions for the robot. Available methods for control (set_joint_velocities, set_joint_position_velocities, set_joint_torques)
        rate.sleep()