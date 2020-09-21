# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


# Copyright 2011-2014, Michael Ferguson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import copy
import sys

if sys.version_info > (3, 0):
    # Python 3 code in this block
    import _thread as thread
else:
    # Python 2 code in this block
    import thread


import rospy

try:
    from pyassimp import pyassimp

    use_pyassimp = True
except:
    # In 16.04, pyassimp is busted
    # https://bugs.launchpad.net/ubuntu/+source/assimp/+bug/1589949
    use_pyassimp = False

from moveit_pybind import Pose, Point
from moveit_pybind import CollisionObject, AttachedCollisionObject, PlanningSceneInterfaceWrapper
from moveit_pybind import MeshTriangle, Mesh, SolidPrimitive, Plane


class PlanningSceneInterface(object):
    """
    A class for managing the state of the planning scene
    """

    def __init__(self, frame, ns="", init_from_service=True):
        """
        Constructor of PlanningSceneInterface class

        :param frame: The fixed frame in which planning is being 
                        done (needs to be part of robot?)
        :param ns: A namespace to push all topics down into.
        :param init_from_service: Whether to initialize our list of objects 
                    by calling the service 
                    NOTE: this requires that said service be in the move_group 
                    launch file, which is not the default from the setup assistant.

        """
        # ns must be a string
        # if not isinstance(ns, basestring):
        #     rospy.logerr('Namespace must be a string!')
        #     ns = ''
        # elif not ns.endswith('/'):
        #     ns += '/'

        self._fixed_frame = frame

        # track the attached and collision objects
        self._mutex = thread.allocate_lock()
        # these are updated based what the planning scene actually contains
        self._attached = list()
        self._collision = list()
        # these are updated based on internal state
        self._objects = dict()
        self._attached_objects = dict()
        self._removed = dict()
        self._attached_removed = dict()
        self._colors = dict()

        self.planning_scene_interface = PlanningSceneInterfaceWrapper()

    def clear(self):
        """
        Clear the planning scene of all objects
        """
        self.planning_scene_interface.removeCollisionObjects(self.planning_scene_interface.getKnownObjectNames())

    def makeMesh(self, name, ps, filename):
        """
        Make a mesh collision object

        :param name: Name of the object
        :param ps: A pose stamped object pose message
        :param filename: The mesh file to load
        """
        if not use_pyassimp:
            rospy.logerr("pyassimp is broken on your platform, cannot load meshes")
            return
        scene = pyassimp.load(filename)
        if not scene.meshes:
            rospy.logerr("Unable to load mesh")
            return

        mesh = Mesh()
        for face in scene.meshes[0].faces:
            triangle = MeshTriangle()
            if len(face.indices) == 3:
                triangle.vertex_indices = [
                    face.indices[0],
                    face.indices[1],
                    face.indices[2],
                ]
            mesh.triangles.append(triangle)
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0]
            point.y = vertex[1]
            point.z = vertex[2]
            mesh.vertices.append(point)
        pyassimp.release(scene)

        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = ps.header.frame_id
        o.id = name
        o.meshes.append(mesh)
        o.mesh_poses.append(ps.pose)
        o.operation = o.ADD
        return o

    def makeSolidPrimitive(self, name, solid, ps):
        """
        Make a solid primitive collision object

        :param name: Name of the object
        :param solid: The solid primitive to add
        :param ps: pose stamped msg containing the object's pose
        """
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = ps.header.frame_id
        o.id = name
        o.primitives.append(solid)
        o.primitive_poses.append(ps.pose)
        o.operation = o.ADD
        return o

    def makeAttached(self, link_name, obj, touch_links, detach_posture, weight):
        """
        Make an attachedCollisionObject
        """
        o = AttachedCollisionObject()
        o.link_name = link_name
        o.object = obj
        if touch_links:
            o.touch_links = touch_links
        if detach_posture:
            o.detach_posture = detach_posture
        o.weight = weight
        return o

    def addMesh(self, name, ps, filename, use_service=True):
        """
        Insert a mesh into the planning scene

        :param name: Name of the object
        :param ps: A pose stamped message for the object
        :param filename: The mesh file to load
        :param use_service: If true, update will be sent via apply service
        """
        o = self.makeMesh(name, ps, filename)
        self.planning_scene_interface.applyCollisionObject(o)

    def attachMesh(
        self,
        name,
        ps,
        filename,
        link_name,
        touch_links=None,
        detach_posture=None,
        weight=0.0,
        use_service=True,
    ):
        """
        Attach a mesh into the planning scene

        :param name: Name of the object
        :param ps: A pose stamped message for the object
        :param filename: The mesh file to load
        :param link_name: Name of link to attach this object to
        :param use_service: If true, update will be sent via apply service
        """
        o = self.makeMesh(name, ps, filename)
        o.header.frame_id = link_name
        a = self.makeAttached(link_name, o, touch_links, detach_posture, weight)
        self.planning_scene_interface.applyAttachedCollisionObject(o)

    def addSolidPrimitive(self, name, solid, ps, use_service=True):
        """
        Insert a solid primitive into planning scene

        :param name: Name of the object
        :param solid: The solid primitive to add
        :param ps: A pose stamped message for the object
        :param use_service: If true, update will be sent via apply service
        """
        o = self.makeSolidPrimitive(name, solid, ps)
        self._objects[name] = o
        self.planning_scene_interface.applyCollisionObject(o)

    def addCylinder(self, name, height, radius, ps, use_service=True):
        """
        Insert new cylinder into planning scene
        """
        s = SolidPrimitive()
        s.dimensions = [height, radius]
        s.type = s.CYLINDER

        self.addSolidPrimitive(name, s, ps, use_service)

    def addBox(self, name, size_x, size_y, size_z, ps, use_service=True):
        """
        Insert new box into planning scene

        :param name: Name of the object
        :param size_x: The x-dimensions size of the box
        :param size_y: The y-dimensions size of the box
        :param size_z: The z-dimensions size of the box
        :param ps: A pose stamped message for the object
        :param use_service: If true, update will be sent via apply service
        """
        s = SolidPrimitive()
        s.dimensions = [size_x, size_y, size_z]
        s.type = s.BOX

        self.addSolidPrimitive(name, s, ps, use_service)

    def attachBox(
        self,
        name,
        size_x,
        size_y,
        size_z,
        pose,
        link_name,
        touch_links=None,
        detach_posture=None,
        weight=0.0,
        use_service=True,
    ):
        """
        Attach a box into the planning scene

        :param name: Name of the object
        :param size_x: The x-dimensions size of the box
        :param size_y: The y-dimensions size of the box
        :param size_z: The z-dimensions size of the box
        :param x: The x position in link_name frame
        :param y: The y position in link_name frame
        :param z: The z position in link_name frame
        :param pose: pose msg containing the pose of the object
        :param link_name: Name of link to attach this object to
        :param touch_links: Names of robot links that can touch this object
        :param use_service: If true, update will be sent via apply service
        """
        s = SolidPrimitive()
        s.dimensions = [size_x, size_y, size_z]
        s.type = s.BOX

        p = Pose()
        p.position.x = pose[0]
        p.position.y = pose[1]
        p.position.z = pose[2]
        p.orientation.x = pose[3]
        p.orientation.y = pose[4]
        p.orientation.z = pose[5]
        p.orientation.w = pose[6]
        o = self.makeSolidPrimitive(name, s, p)
        o.header.frame_id = link_name
        a = self.makeAttached(link_name, o, touch_links, detach_posture, weight)
        self.planning_scene_interface.applyAttachedCollisionObject(o)

    def removeCollisionObject(self, name, use_service=True):
        """
        Removes an object from the scene
        
        :param name: name of the object to be removed
        :param use_service: If true, update will be sent via apply service
        """
        self.planning_scene_interface.removeCollisionObjects([name])


    def getKnownCollisionObjects(self):
        """
        Get a list of names of collision objects
        """
        return self.planning_scene_interface.getObjects()

    def getKnownAttachedObjects(self):
        """
        Get a list of names of attached objects
        """
        return self.planning_scene_interface.getAttachedObjects()

