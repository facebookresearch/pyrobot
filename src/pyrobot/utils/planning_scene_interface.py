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

from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane


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

        self._scene_pub = rospy.Publisher(
            ns + "planning_scene", PlanningScene, queue_size=10
        )
        self._apply_service = rospy.ServiceProxy(
            ns + "apply_planning_scene", ApplyPlanningScene
        )
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

        # get the initial planning scene
        if init_from_service:
            rospy.loginfo("Waiting for get_planning_scene")
            rospy.wait_for_service(ns + "get_planning_scene")
            self._service = rospy.ServiceProxy(
                ns + "get_planning_scene", GetPlanningScene
            )
            try:
                req = PlanningSceneComponents()
                req.components = sum(
                    [
                        PlanningSceneComponents.WORLD_OBJECT_NAMES,
                        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
                        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
                    ]
                )
                scene = self._service(req)
                self.sceneCb(scene.scene, initial=True)
            except rospy.ServiceException as e:
                rospy.logerr(
                    "Failed to get initial planning scene, results may be wonky: %s", e
                )

        # subscribe to planning scene
        rospy.Subscriber(
            ns + "move_group/monitored_planning_scene", PlanningScene, self.sceneCb
        )

    def sendUpdate(self, collision_object, attached_collision_object, use_service=True):
        """
        Send new update to planning scene

        :param collision_object: A collision object to add to scene, or None
        :param attached_collision_object: Attached collision object to add to scene, or None
        :param use_service: If true, update will be sent via apply service, otherwise by topic
        """
        ps = PlanningScene()
        ps.is_diff = True
        if collision_object:
            ps.world.collision_objects.append(collision_object)

        if attached_collision_object:
            ps.robot_state.attached_collision_objects.append(attached_collision_object)

        if use_service:
            resp = self._apply_service.call(ps)
            if not resp.success:
                rospy.logerr("Could not apply planning scene diff.")
        else:
            self._scene_pub.publish(ps)

    def clear(self):
        """
        Clear the planning scene of all objects
        """
        for name in self.getKnownCollisionObjects():
            self.removeCollisionObject(name, True)
        for name in self.getKnownAttachedObjects():
            self.removeAttachedObject(name, True)
        self.waitForSync()

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
        self._objects[name] = o
        self.sendUpdate(o, None, use_service)

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
        self._attached_objects[name] = a
        self.sendUpdate(None, a, use_service)

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
        self.sendUpdate(o, None, use_service)

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
        self._attached_objects[name] = a
        self.sendUpdate(None, a, use_service)

    def removeCollisionObject(self, name, use_service=True):
        """
        Removes an object from the scene
        
        :param name: name of the object to be removed
        :param use_service: If true, update will be sent via apply service
        """
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self._fixed_frame
        o.id = name
        o.operation = o.REMOVE

        try:
            del self._objects[name]
            self._removed[name] = o
        except KeyError:
            pass

        self.sendUpdate(o, None, use_service)

    def removeAttachedObject(self, name, use_service=True):
        """
        Removes an attached object. 
        
        :param name: name of the object to be removed
        :param use_service: If true, update will be sent via apply service
        """
        o = AttachedCollisionObject()
        o.object.operation = CollisionObject.REMOVE
        o.object.id = name

        try:
            del self._attached_objects[name]
            self._attached_removed[name] = o
        except KeyError:
            pass

        self.sendUpdate(None, o, use_service)

    def sceneCb(self, msg, initial=False):
        """
        Update the object lists from a PlanningScene message
        """
        # Recieve updates from move_group
        self._mutex.acquire()
        for obj in msg.world.collision_objects:
            try:
                if obj.operation == obj.ADD:
                    self._collision.append(obj.id)
                    rospy.logdebug('ObjectManager: Added Collision Obj "%s"', obj.id)
                    if initial:
                        # this is our initial planning scene, hold onto each object
                        self._objects[obj.id] = obj
                elif obj.operation == obj.REMOVE:
                    self._collision.remove(obj.id)
                    self._removed.pop(obj.id, None)
                    rospy.logdebug('ObjectManager: Removed Collision Obj "%s"', obj.id)
            except ValueError:
                pass
        self._attached = list()
        for obj in msg.robot_state.attached_collision_objects:
            rospy.logdebug('ObjectManager: attached collision Obj "%s"', obj.object.id)
            self._attached.append(obj.object.id)
            if initial:
                # this is our initial planning scene, hold onto each object
                self._attached_objects[obj.object.id] = obj
        self._mutex.release()

    def getKnownCollisionObjects(self):
        """
        Get a list of names of collision objects
        """
        self._mutex.acquire()
        l = copy.deepcopy(self._collision)
        self._mutex.release()
        return l

    def getKnownAttachedObjects(self):
        """
        Get a list of names of attached objects
        """
        self._mutex.acquire()
        l = copy.deepcopy(self._attached)
        self._mutex.release()
        return l

    def waitForSync(self, max_time=2.0):
        """
        Clears and syncs the planning scene
        """
        sync = False
        t = rospy.Time.now()
        while not sync:
            sync = True
            # delete objects that should be gone
            for name in self._collision:
                if name in self._removed.keys():
                    # should be removed, is not
                    rospy.logwarn("ObjectManager: %s not removed yet", name)
                    self.removeCollisionObject(name, False)
                    sync = False
            for name in self._attached:
                if name in self._attached_removed.keys():
                    # should be removed, is not
                    rospy.logwarn(
                        "ObjectManager: Attached object name: %s not removed yet", name
                    )
                    self.removeAttachedObject(name, False)
                    sync = False
            # add missing objects
            for name in self._objects.keys():
                if name not in self._collision + self._attached:
                    rospy.logwarn("ObjectManager: %s not added yet", name)
                    self.sendUpdate(self._objects[name], None, False)
                    sync = False
            for name in self._attached_objects.keys():
                if name not in self._attached:
                    rospy.logwarn("ObjectManager: %s not attached yet", name)
                    self.sendUpdate(None, self._attached_objects[name], False)
                    sync = False
            # timeout
            if rospy.Time.now() - t > rospy.Duration(max_time):
                rospy.logerr("ObjectManager: sync timed out.")
                break
            rospy.logdebug("ObjectManager: waiting for sync.")
            rospy.sleep(0.1)

    def setColor(self, name, r, g, b, a=0.9):
        """
        Set the color of the specified object
        """
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self._colors[name] = color

    def sendColors(self):
        """
        Send the colors set using 'setColor' to moveit.
        """
        p = PlanningScene()
        p.is_diff = True
        for color in self._colors.values():
            p.object_colors.append(color)
        resp = self._apply_service.call(p)
        if not resp.success:
            rospy.logerr(
                "Could not update colors through service, using topic instead."
            )
            self._scene_pub.publish(p)
