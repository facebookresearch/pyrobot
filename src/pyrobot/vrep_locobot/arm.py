import numpy as np
import math
import pyrobot.utils.util as prutil
import rospy
from tf.transformations import euler_from_quaternion, euler_from_matrix

from pyrep.robots.mobiles.nonholonomic_base import NonHolonomicBase
from pyrep.errors import ConfigurationPathError
from pyrep.robots.arms.arm import Arm
import pyrobot.utils.util as prutil
from pyrep.objects.object import Object

from pyrep.errors import ConfigurationError, ConfigurationPathError, IKError
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
import copy


class LoCoBotArm(object):
    """docstring for SimpleBase"""

    def __init__(self, configs, simulator):
        self.configs = configs

        self.sim = simulator.sim

        ## Vrep objects
        self.arm = Arm(0, "LoCoBotArm", 5)

        # arm_base_link

        self.arm_base_link = Shape("LoCoBot_clear_plate_visual")
        self.ee_link = self.arm.get_tip()

        # todo: add remapping for joint names and frames to vrep

    def compute_fk_position(self, joint_positions):
        """
		Given joint angles, compute the pose of desired_frame with respect
		to the base frame (self.configs.ARM.ARM_BASE_FRAME). The desired frame
		must be in self.arm_link_names

		:param joint_positions: joint angles
		:param des_frame: desired frame
		:type joint_positions: np.ndarray
		:type des_frame: string
		:return: translational vector and rotational matrix
		:rtype: np.ndarray, np.ndarray
		"""
        # raise NotImplementedError
        pos, quat = self.ee_link.get_position(), self.ee_link.get_quaternion()
        pos = np.asarray(pos)
        rot = prutil.quat_to_rot_mat(quat)
        return pos, rot

    def compute_fk_velocity(self, joint_positions, joint_velocities, des_frame):
        """
		Given joint_positions and joint velocities,
		compute the velocities of des_frame with respect
		to the base frame

		:param joint_positions: joint positions
		:param joint_velocities: joint velocities
		:param des_frame: end frame
		:type joint_positions: np.ndarray
		:type joint_velocities: np.ndarray
		:type des_frame: string
		:return: translational and rotational
		         velocities (vx, vy, vz, wx, wy, wz)
		:rtype: np.ndarray
		"""
        raise NotImplementedError

    def _quaternion_multiply(self, quaternion1, quaternion0):
        x0, y0, z0, w0 = quaternion0
        x1, y1, z1, w1 = quaternion1
        return np.array(
            [
                x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
            ],
            dtype=np.float64,
        )

    def compute_ik(self, position, orientation, qinit=None, numerical=True):
        """
		Inverse kinematics
		computes joints for robot arm to desired end-effector pose
        (w.r.t. 'ARM_BASE_FRAME').
		:param position: end effector position (shape: :math:`[3,]`)
		:param orientation: end effector orientation.
		                    It can be quaternion ([x,y,z,w],
		                    shape: :math:`[4,]`),
		                    euler angles (yaw, pitch, roll
		                    angles (shape: :math:`[3,]`)),
		                    or rotation matrix (shape: :math:`[3, 3]`)
		:param qinit: initial joint positions for numerical IK
		:param numerical: use numerical IK or analytical IK
		:type position: np.ndarray
		:type orientation: np.ndarray
		:type qinit: list
		:type numerical: bool
		:return: None or joint positions
		:rtype: np.ndarray
		"""

        position = position.flatten()

        if orientation.size == 4:
            quat = orientation.flatten()
        elif orientation.size == 3:
            quat = prutil.euler_to_quat(orientation)
        elif orientation.size == 9:
            quat = prutil.rot_mat_to_quat(orientation)
        else:
            raise TypeError(
                "Orientation must be in one "
                "of the following forms:"
                "rotation matrix, euler angles, or quaternion"
            )

        # This is needed to conform compute_ik to core.py's compute ik

        quat_world_to_base_link = np.asarray(self.arm_base_link.get_quaternion())
        quat = self._quaternion_multiply(quat_world_to_base_link, quat)

        rot_world_to_base_link = np.transpose(
            prutil.quat_to_rot_mat(np.asarray(self.arm_base_link.get_quaternion()))
        )
        position = np.matmul(position, rot_world_to_base_link)
        position = position + np.asarray(self.arm_base_link.get_position())

        print(position, prutil.quat_to_rot_mat(quat))
        # print(position, quat)
        try:
            joint_angles = self.arm.solve_ik(
                position.tolist(), euler=None, quaternion=quat.tolist()
            )
        except IKError as error:
            print(error)
            return None
        joint_angles = np.asarray(joint_angles)
        return joint_angles

    @property
    def pose_ee(self):
        """
		Return the end effector pose w.r.t 'ARM_BASE_FRAME'

		:return:
		        trans: translational vector (shape: :math:`[3, 1]`)

		        rot_mat: rotational matrix (shape: :math:`[3, 3]`)

		        quat: rotational matrix in the form
		        of quaternion (shape: :math:`[4,]`)

		:rtype: tuple (trans, rot_mat, quat)
		"""
        pos = (self.ee_link.get_position(self.arm_base_link),)
        quat = self.ee_link.get_quaternion(self.arm_base_link)

        # print("#################Debug start########################")
        # print(quat)
        # print("#################Debug end########################")

        pos = np.asarray(pos).flatten()
        quat = np.asarray(quat).flatten()
        rot = prutil.quat_to_rot_mat(quat)
        return pos, rot, quat

    def get_ee_pose(self, base_frame=None):
        """
		Return the end effector pose with respect to the base_frame

		:param base_frame: reference frame
		:type base_frame: string
		:return:
		        tuple (trans, rot_mat, quat)

		        trans: translational vector (shape: :math:`[3, 1]`)

		        rot_mat: rotational matrix (shape: :math:`[3, 3]`)

		        quat: rotational matrix in the form
		        of quaternion (shape: :math:`[4,]`)

		:rtype: tuple or ROS PoseStamped
		"""
        # raise NotImplementedError
        pos = (self.ee_link.get_position(),)
        quat = self.ee_link.get_quaternion()
        pos = np.asarray(pos).flatten()
        quat = np.asarray(quat).flatten()
        rot = prutil.quat_to_rot_mat(quat)
        return pos, rot, quat

    def go_home(self, plan=False):
        """
		Commands robot to home position

		:param plan: use moveit to plan the path or not
		:type plan: bool
		"""
        return self.set_joint_positions(np.zeros(self.arm_dof), plan=plan)

    def get_transform(self, src_frame, dest_frame):
        """
		Return the transform from the src_frame to dest_frame

		:param src_frame: source frame
		:param dest_frame: destination frame
		:type src_frame: string
		:type dest_frame: basestring
		:return:
				tuple (trans, rot_mat, quat )

				trans: translational vector (shape: :math:`[3, 1]`)

				rot_mat: rotational matrix (shape: :math:`[3, 3]`)

				quat: rotational matrix in the form
				of quaternion (shape: :math:`[4,]`)

		:rtype: tuple or ROS PoseStamped
		"""
        raise NotImplementedError

    def get_joint_angles(self):
        """
		Return the joint angles

		:return: joint_angles
		:rtype: np.ndarray
		"""
        joint_angles = self.arm.get_joint_positions()
        joint_angles = np.asarray(joint_angles).flatten()
        return joint_angles

    def get_joint_velocities(self):
        """
		Return the joint velocities

		:return: joint_vels
		:rtype: np.ndarray
		"""
        joint_vels = self.arm.get_joint_velocities()
        joint_vels = np.asarray(joint_vels).flatten()
        return joint_vels

    def get_joint_torques(self):
        """
		Return the joint torques

		:return: joint_torques
		:rtype: np.ndarray
		"""
        joint_torqs = self.arm.get_joint_forces()
        joint_torqs = np.asarray(joint_torqs).flatten()
        return joint_torqs

    def get_joint_angle(self, joint):
        """
		Return the joint angle of the 'joint'

		:param joint: joint name
		:type joint: string
		:return: joint angle
		:rtype: float
		"""
        raise NotImplementedError

    def get_joint_velocity(self, joint):
        """
		Return the joint velocity of the 'joint'

		:param joint: joint name
		:type joint: string
		:return: joint velocity
		:rtype: float
		"""
        raise NotImplementedError

    def get_joint_torque(self, joint):
        """
		Return the joint torque of the 'joint'

		:param joint: joint name
		:type joint: string
		:return: joint torque
		:rtype: float
		"""
        raise NotImplementedError

    def set_joint_velocities(self, velocities, **kwargs):
        """
		Sets the desired joint velocities for all arm joints

		:param velocities: target joint velocities
		:type velocities: list
		"""
        raise NotImplementedError

    def set_joint_torques(self, torques, **kwargs):
        """
		Sets the desired joint torques for all arm joints

		:param torques: target joint torques
		:type torques: list
		"""
        raise NotImplementedError

    def set_joint_positions(self, positions, plan=False, wait=True, **kwargs):
        """
		Sets the desired joint angles for all arm joints

		:param positions: list of length #of joints, angles in radians
		:param plan: whether to use moveit to plan a path. Without planning,
		             there is no collision checking and each joint will
		             move to its target joint position directly.
		:param wait: if True, it will wait until the desired
		             joint positions are achieved
		:type positions: list
		:type plan: bool
		:type wait: bool

		:return: True if successful; False otherwise (timeout, etc.)
		:rtype: bool
		"""
        if isinstance(positions, np.ndarray):
            positions = positions.flatten().tolist()
        if plan:
            raise NotImplementedError
            pos, rot = self.compute_fk_position(positions)
            return self.set_ee_pose(pos, rot, plan=True)
        else:
            self.arm.set_joint_positions(positions)
            return True

    def set_ee_pose(
        self, position, orientation, plan=True, wait=True, numerical=True, **kwargs
    ):
        """
		Commands robot arm to desired end-effector pose
		(w.r.t. 'ARM_BASE_FRAME').
		Computes IK solution in joint space and calls set_joint_positions.
		Will wait for command to complete if wait is set to True.

		:param position: position of the end effector (shape: :math:`[3,]`)
		:param orientation: orientation of the end effector
		                    (can be rotation matrix, euler angles (yaw,
		                    pitch, roll), or quaternion)
		                    (shape: :math:`[3, 3]`, :math:`[3,]`
		                    or :math:`[4,]`)
		                    The convention of the Euler angles here
		                    is z-y'-x' (intrinsic rotations),
		                    which is one type of Tait-Bryan angles.
		:param plan: use moveit the plan a path to move to the desired pose
		:param wait: wait until the desired pose is achieved
		:param numerical: use numerical inverse kinematics solver or
		                  analytical inverse kinematics solver
		:type position: np.ndarray
		:type orientation: np.ndarray
		:type plan: bool
		:type wait: bool
		:type numerical: bool
		:return: Returns True if command succeeded, False otherwise
		:rtype: bool
		"""

        if orientation.size == 4:
            quat = orientation.flatten()
        elif orientation.size == 3:
            quat = prutil.euler_to_quat(orientation)
        elif orientation.size == 9:
            quat = prutil.rot_mat_to_quat(orientation)

        quat_world_to_base_link = np.asarray(self.arm_base_link.get_quaternion())
        quat = self._quaternion_multiply(quat_world_to_base_link, quat)

        rot_world_to_base_link = np.transpose(
            prutil.quat_to_rot_mat(np.asarray(self.arm_base_link.get_quaternion()))
        )
        position = np.matmul(position, rot_world_to_base_link)
        position = position + np.asarray(self.arm_base_link.get_position())

        try:
            arm_path = self.arm.get_path(
                position.tolist(), quaternion=quat.tolist(), ignore_collisions=False
            )
        except ConfigurationPathError as error:
            print(error)
            return False
        # arm_path.visualize()
        # done = False
        # while not done:
        # 	done = arm_path.step()
        # 	pr.step()
        # arm_path.clear_visualization()
        arm_path.set_to_end()
        return True

    def move_ee_xyz(
        self, displacement, eef_step=0.005, numerical=True, plan=False, **kwargs
    ):
        """
		Keep the current orientation fixed, move the end
		effector in {xyz} directions

		:param displacement: (delta_x, delta_y, delta_z)
		:param eef_step: resolution (m) of the interpolation
		                 on the cartesian path
		:param numerical: use numerical inverse kinematics solver or
		                  analytical inverse kinematics solver
		:param plan: use moveit the plan a path to move to the
		             desired pose. If False,
		             it will do linear interpolation along the path,
		             and simply use IK solver to find the
		             sequence of desired joint positions and
		             then call `set_joint_positions`
		:type displacement: np.ndarray
		:type eef_step: float
		:type numerical: bool
		:type plan: bool
		:return: whether the movement is successful or not
		:rtype: bool
		"""
        displacement = displacement.reshape(-1, 1)

        path_len = np.linalg.norm(displacement)
        num_pts = int(np.ceil(path_len / float(eef_step)))
        if num_pts <= 1:
            num_pts = 2

        cur_pos, cur_ori, cur_quat = self.pose_ee
        waypoints_sp = np.linspace(0, path_len, num_pts)
        cur_pos = cur_pos.reshape(-1, 1)
        waypoints = cur_pos + waypoints_sp / float(path_len) * displacement
        way_joint_positions = []
        qinit = self.get_joint_angles().tolist()
        for i in range(waypoints.shape[1]):
            joint_positions = self.compute_ik(
                waypoints[:, i].flatten(), cur_quat, qinit=qinit, numerical=numerical
            )
            if joint_positions is None:
                rospy.logerr("No IK solution found; " "check if target_pose is valid")
                return False
            way_joint_positions.append(copy.deepcopy(joint_positions))
            qinit = copy.deepcopy(joint_positions)
        success = False
        for joint_positions in way_joint_positions:
            success = self.set_joint_positions(joint_positions, plan=plan, wait=True)

        return success

    def set_ee_pose_pitch_roll(
        self, position, pitch, roll=None, plan=True, wait=True, numerical=True, **kwargs
    ):
        """
		Commands robot arm to desired end-effector pose
		(w.r.t. 'ARM_BASE_FRAME').
		Computes IK solution in joint space and calls set_joint_positions.
		Will wait for command to complete if wait is set to True.

		:param position: position of the end effector (shape: :math:`[3,]`)
		:param pitch: pitch angle
		:param roll: roll angle
		:param plan: use moveit the plan a path to move to the desired pose
		:param wait: wait until the desired pose is achieved
		:param numerical: use numerical inverse kinematics solver or
		                  analytical inverse kinematics solver
		:type position: np.ndarray
		:type pitch: float
		:type roll: float
		:type plan: bool
		:type wait: bool
		:type numerical: bool
		:return result: Returns True if command succeeded, False otherwise
		:rtype: bool
		"""
        raise NotImplementedError

        position = np.array(position).flatten()
        base_offset, _, _ = self.get_transform(
            self.configs.ARM.ARM_BASE_FRAME, "arm_base_link"
        )
        yaw = np.arctan2(position[1] - base_offset[1], position[0] - base_offset[0])
        if roll is None:
            # read the current roll angle
            roll = -self.get_joint_angle("joint_5")
        euler = np.array([yaw, pitch, roll])
        return self.set_ee_pose(
            position=position,
            orientation=euler,
            plan=plan,
            wait=wait,
            numerical=numerical,
            **kwargs
        )
