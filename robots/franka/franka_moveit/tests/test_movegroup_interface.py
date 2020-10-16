import rospy
from franka_moveit import PandaMoveGroupInterface

if __name__ == '__main__':
    

    rospy.init_node("test_moveit")
    mvt = PandaMoveGroupInterface()

    g = mvt.gripper_group
    r = mvt.arm_group


