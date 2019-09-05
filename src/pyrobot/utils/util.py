# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import sys
import numpy as np
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from pyrobot.utils.planning_scene_interface import PlanningSceneInterface



def list_to_pose(pose_list):
    pose_msg = Pose()
    if len(pose_list) == 7:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]
    elif len(pose_list) == 6: 
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        q = tf.transformations.quaternion_from_euler(
                                            pose_list[3],
                                            pose_list[4], 
                                            pose_list[5])
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
    return pose_msg

def get_tf_transform(tf_listener, tgt_frame, src_frame):
    """
    Uses ROS TF to lookup the current transform from tgt_frame to src_frame,
    If the returned transform is applied to data, it will transform data in
    the src_frame into the tgt_frame

    :param tgt_frame: target frame
    :param src_frame: source frame
    :type tgt_frame: string
    :type src_frame: string

    :returns: trans, translation (x,y,z)
    :rtype: tuple (of floats)
    :returns: quat, rotation as a quaternion (x,y,z,w)
    :rtype: tuple (of floats)
    """
    try:
        tf_listener.waitForTransform(tgt_frame, src_frame,
                                     rospy.Time(0),
                                     rospy.Duration(3))
        (trans, quat) = tf_listener.lookupTransform(tgt_frame,
                                                    src_frame,
                                                    rospy.Time(0))
    except (tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException):
        raise RuntimeError('Cannot fetch the transform from'
                           ' {0:s} to {1:s}'.format(tgt_frame, src_frame))
    return trans, quat

def quat_to_rot_mat(quat):
    """
    Convert the quaternion into rotation matrix. The quaternion we used
    here is in the form of [x, y, z, w]

    :param quat: quaternion [x, y, z, w] (shape: :math:`[4,]`)
    :type quat: numpy.ndarray

    :return: the rotation matrix (shape: :math:`[3, 3]`)
    :rtype: numpy.ndarray
    """
    return tf.transformations.quaternion_matrix(quat)[:3, :3]


def euler_to_quat(euler):
    """
    Convert the yaw, pitch, roll into quaternion.

    :param euler: the yaw, pitch, roll angles (shape: :math:`[3,]`)
    :type quat: numpy.ndarray

    :return: quaternion [x, y, z, w] (shape: :math:`[4,]`)
    :rtype: numpy.ndarray
    """
    return tf.transformations.quaternion_from_euler(euler[0], euler[1],
                                                    euler[2], axes='rzyx')


def rot_mat_to_quat(rot):
    """
    Convert the rotation matrix into quaternion.

    :param quat: the rotation matrix (shape: :math:`[3, 3]`)
    :type quat: numpy.ndarray

    :return: quaternion [x, y, z, w] (shape: :math:`[4,]`)
    :rtype: numpy.ndarray
    """
    R = np.eye(4)
    R[:3, :3] = rot
    return tf.transformations.quaternion_from_matrix(R)


class MoveitObjectHandler(object):
    '''
    Use this class to create objects that reside in moveit environments
    '''
    def __init__(self, frame='/base_link'):
        '''
        Constructor of the MoveitObjectHandler class.
        '''
        self.planning_scene_interface = PlanningSceneInterface(frame)
        self.scene_objects = []
        self.attached_objects = []
        
    def add_world_object(self, id_name, pose, size, frame='/base_link'):
        '''
        Adds the particular BOX TYPE objects to the moveit planning scene
        
        :param id_name: unique id that object should be labeled with
        :param pose: pose of the object
        :param size: size of the object
        :param frame: frame in which the object pose is passed

        :type id_name: string
        :type pose: list of double of length 7 (x,y,z, q_x, q_y, q_z, q_w)
        :type size: tuple of length 3
        :type frame: string
        '''
        assert type(size) is tuple, 'size should be tuple'
        assert len(size)==3, 'size should be of length 3'
        assert not id_name in self.scene_objects, \
                                'Object with the same name already exists!'
        self.scene_objects.append(id_name)

        pose = list_to_pose(pose)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame
        pose_stamped.pose = pose

        self.planning_scene_interface.addBox(id_name, size[0], size[1], 
                                             size[2], pose_stamped)

    def remove_world_object(self, id_name):
        '''
        Removes a specified object for the Moveit planning scene

        :param id_name: unique id that object should be labeled with
        :type frame: string

        ''' 
        assert id_name in self.scene_objects, 'Incorrect object name!'
        self.scene_objects.remove(id_name)
        self.planning_scene_interface.removeCollisionObject(id_name)

    def attach_arm_object(self, link_name, id_name, pose, size):
        '''
        Attaches the specified Box type object to the robot

        :param link_name: name of the link to which the bject 
                          should be attached
        :param id_name: unique id associated with the object
        :param pose: pose of the object
        :parma size: size of the object
        
        :type link_name: string
        :type id_name: string
        :type pose: list of double of length 7 (x,y,z, q_x, q_y, q_z, q_w)
        :type size: tuple of length 3
        '''
        assert type(size) is tuple, 'size should be tuple'
        assert len(size)==3, 'size should be of length 3'
        assert not id_name in self.attached_objects, \
                                'Object with the same name already exists!'
        self.scene_objects.append(id_name)
        self.attached_objects.append(id_name)

        self.planning_scene_interface.attachBox(id_name, size[0], size[1], size[2],
                                                pose, link_name)        

    def detach_arm_object(self, link_name, id_name, remove_from_world=True):
        '''
        Detaches an object earlier attached to the robot

        :param link_name: name of the link from which the bject 
                          should be detached
        :param id_name: unique id associated with the object
        :param remove_from_world: if set true, deletes the 
                                  object from the scene.
        
        :type link_name: string
        :type id_name: string
        :type remove_from_world: bool
        '''
        assert id_name in self.attached_objects, 'Incorrect object name!'
        self.planning_scene_interface.remove_attached_object(link_name, id_name)
        self.attached_objects.remove(id_name)

        if remove_from_world is True:
            self.remove_world_object(id_name)

    def remove_all_objects(self):
        '''
        Removes all the objects in the current Moveit planning scene
        '''
        ## get add objects
        dict_obj = self.scene.get_objects()
        ## get attach object
        dict_attach_obj = self.scene.get_attached_objects()
        ## remove add objects
        for i in dict_obj.keys():
            self.remove_world_object(i)
        ## remove attached objects
        for i in dict_attach_obj.keys():
            self.detach_arm_object(dict_attach_obj[i].link_name,i)

    def add_table(self, pose=None, size=None):
        '''
        Adds a table in the planning scene in the base frame.
        
        :param pose: pose of the object
        :parma size: size of the object
        
        
        :type pose: list of double of length 7 (x,y,z, q_x, q_y, q_z, q_w)
        :type size: tuple of length 3
        '''
        if pose is not None and size is not None:
            self.add_world_object('table', 
                pose=pose, 
                size=size)
        else:
            # Default table.
            print('Creating default table.')
            self.add_world_object('table', 
                pose=[0.8,0.0,-0.23,0.,0.,0.,1.],
                size=(1.35,2.0,0.1))

    def add_kinect(self, pose=None, size=None):
        '''
        Adds a kinect object to the planning scene in the base frame.

        :param pose: pose of the object
        :parma size: size of the object
        
        
        :type pose: list of double of length 7 (x,y,z, q_x, q_y, q_z, q_w)
        :type size: tuple of length 3        
        '''
        if pose is not None and size is not None:
            self.add_world_object('kinect', 
                pose=pose, 
                size=size)
        else:
            # Default kinect.
            print('Creating default kinect.')
            self.add_world_object('kinect', 
                pose=[0., 0.0,0.75,0.,0.,0.,1.], 
                size=(0.25,0.25,0.3))

    def add_gripper(self, pose=None, size=None):
        '''
        Attaches gripper object to 'gripper' link.
        
        :param pose: pose of the object
        :param size: size of the object
        
        
        :type pose: list of double of length 7 (x,y,z, q_x, q_y, q_z, q_w)
        :type size: tuple of length 3
        '''
        if pose is not None and size is not None:
            self.attach_arm_object('right_gripper',
                'gripper',  
                pose=pose, 
                size=size)
        else:
            # Default gripper.
            print('Creating default gripper.')
            self.attach_arm_object('right_gripper',
                'gripper', 
                pose=[0., 0.0, 0.07,0.,0.,0.,1.], 
                size=(0.02,0.1,0.07))

    def remove_table(self):
        '''
        Removes table object from the planning scene
        '''
        self.remove_world_object('table')

    def remove_gripper(self):
        '''
        Removes table object from the planning scene
        '''
        self.detach_object('gripper')
        rospy.sleep(0.2)
        self.detach_object('gripper')
        rospy.sleep(0.2)
        self.remove_world_object('gripper')
        rospy.sleep(0.2)
        self.remove_world_object('gripper')
        

