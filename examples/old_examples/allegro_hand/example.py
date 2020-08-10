import numpy as np
from pyrobot import Robot

allegro_hand = Robot("allegro_hand")

print("Openning Gripper...")
allegro_hand.gripper.open()
print("Closing Gripper...")
allegro_hand.gripper.close()

print("Openning Gripper...")
allegro_hand.gripper.open()

print("Setting joint positions on Gripper...")
joint_angles = 0.1 * np.ones(allegro_hand.gripper.gripper_dof)
allegro_hand.gripper.set_joint_positions(joint_angles)

print("Reading joint positions on Gripper...")
joint_angles_read = allegro_hand.gripper.get_joint_angles()
print("Joint positions: {}".format(joint_angles_read))

print("Reading joint velocities on Gripper...")
joint_velocities_read = allegro_hand.gripper.get_joint_velocities()
print("Joint Velocities: {}".format(joint_velocities_read))

print("Prepping gripper to perform pinch motion..")
allegro_hand.gripper.set_primitive("ready")
print("Executing pinch motion..")
allegro_hand.gripper.set_primitive("pinch_it")

print("Setting gripper in gravity compensation mode..")
allegro_hand.gripper.set_primitive("gravcomp")
