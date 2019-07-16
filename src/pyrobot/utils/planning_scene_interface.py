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

import thread, copy
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

## @brief A class for managing the state of the planning scene
## @param frame The fixed frame in which planning is being done (needs to be part of robot?)
## @param ns A namespace to push all topics down into.
## @param init_from_service Whether to initialize our list of objects by calling the service
##            NOTE: this requires that said service be in the move_group launch file, which
##            is not the default from the setup assistant.
class PlanningSceneInterface(object):
    def __init__(self, frame, ns='', init_from_service=True):
        # ns must be a string
        if not isinstance(ns, basestring):
            rospy.logerr('Namespace must be a string!')
            ns = ''
        elif not ns.endswith('/'):
            ns += '/'

        self._fixed_frame = frame

        self._scene_pub = rospy.Publisher(ns + 'planning_scene',
                                          PlanningScene,
                                          queue_size=10)
        self._apply_service = rospy.ServiceProxy(ns + 'apply_planning_scene', ApplyPlanningScene)
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
            rospy.loginfo('Waiting for get_planning_scene')
            rospy.wait_for_service(ns + 'get_planning_scene')
            self._service = rospy.ServiceProxy(ns + 'get_planning_scene',
                                               GetPlanningScene)
            try:
                req = PlanningSceneComponents()
                req.components = sum([
                    PlanningSceneComponents.WORLD_OBJECT_NAMES,
                    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
                    PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS])
                scene = self._service(req)
                self.sceneCb(scene.scene, initial=True)
            except rospy.ServiceException as e:
                rospy.logerr('Failed to get initial planning scene, results may be wonky: %s', e)

        # subscribe to planning scene
        rospy.Subscriber(ns + 'move_group/monitored_planning_scene',
                         PlanningScene,
                         self.sceneCb)

    ## @brief Send new update to planning scene
    ## @param collision_object A collision object to add to scene, or None
    ## @param attached_collision_object Attached collision object to add to scene, or None
    ## @param use_service If true, update will be sent via apply service, otherwise by topic
    def sendUpdate(self, collision_object, attached_collision_object, use_service=True):
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

    ## @brief Clear the planning scene of all objects
    def clear(self):
        for name in self.getKnownCollisionObjects():
            self.removeCollisionObject(name, True)
        for name in self.getKnownAttachedObjects():
            self.removeAttachedObject(name, True)
        self.waitForSync()

    ## @brief Make a mesh collision object
    ## @param name Name of the object
    ## @param pose A geometry_msgs/Pose for the object
    ## @param filename The mesh file to load
    def makeMesh(self, name, pose, filename):
        if not use_pyassimp:
            rospy.logerr('pyassimp is broken on your platform, cannot load meshes')
            return
        scene = pyassimp.load(filename)
        if not scene.meshes:
            rospy.logerr('Unable to load mesh')
            return

        mesh = Mesh()
        for face in scene.meshes[0].faces:
            triangle = MeshTriangle()
            if len(face.indices) == 3:
                triangle.vertex_indices = [face.indices[0],
                                           face.indices[1],
                                           face.indices[2]]
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
        o.header.frame_id = self._fixed_frame
        o.id = name
        o.meshes.append(mesh)
        o.mesh_poses.append(pose)
        o.operation = o.ADD
        return o

    ## @brief Make a solid primitive collision object
    ## @param name Name of the object
    ## @param solid The solid primitive to add
    ## @param pose A geometry_msgs/Pose for the object
    def makeSolidPrimitive(self, name, solid, pose):
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self._fixed_frame
        o.id = name
        o.primitives.append(solid)
        o.primitive_poses.append(pose)
        o.operation = o.ADD
        return o

    ## @brief Make an attachedCollisionObject
    def makeAttached(self, link_name, obj, touch_links, detach_posture,
                     weight):
        o = AttachedCollisionObject()
        o.link_name = link_name
        o.object = obj
        if touch_links:
            o.touch_links = touch_links
        if detach_posture:
            o.detach_posture = detach_posture
        o.weight = weight
        return o

    ## @brief Insert a mesh into the planning scene
    ## @param name Name of the object
    ## @param pose A geometry_msgs/Pose for the object
    ## @param filename The mesh file to load
    ## @param use_service If true, update will be sent via apply service
    def addMesh(self, name, pose, filename, use_service=True):
        o = self.makeMesh(name, pose, filename)
        self._objects[name] = o
        self.sendUpdate(o, None, use_service)

    ## @brief Attach a mesh into the planning scene
    ## @param name Name of the object
    ## @param pose A geometry_msgs/Pose for the object
    ## @param filename The mesh file to load
    ## @param use_service If true, update will be sent via apply service
    def attachMesh(self, name, pose, filename, link_name, touch_links=None,
                   detach_posture=None, weight=0.0, use_service=True):
        o = self.makeMesh(name, pose, filename)
        o.header.frame_id = link_name
        a = self.makeAttached(link_name, o, touch_links, detach_posture,
                              weight)
        self._attached_objects[name] = a
        self.sendUpdate(None, a, use_service)

    ## @brief Insert a solid primitive into planning scene
    ## @param use_service If true, update will be sent via apply service
    def addSolidPrimitive(self, name, solid, pose, use_service=True):
        o = self.makeSolidPrimitive(name, solid, pose)
        self._objects[name] = o
        self.sendUpdate(o, None, use_service)

    ## @brief Insert new cylinder into planning scene
    ## @param use_service If true, update will be sent via apply service
    def addCylinder(self, name, height, radius, x, y, z, use_service=True):
        s = SolidPrimitive()
        s.dimensions = [height, radius]
        s.type = s.CYLINDER

        ps = PoseStamped()
        ps.header.frame_id = self._fixed_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0

        self.addSolidPrimitive(name, s, ps.pose, use_service)

    ## @brief Insert new box into planning scene
    ## @param name Name of the object
    ## @param size_x The x-dimensions size of the box
    ## @param size_y The y-dimensions size of the box
    ## @param size_z The z-dimensions size of the box
    ## @param x The x position in link_name frame
    ## @param y The y position in link_name frame
    ## @param z The z position in link_name frame
    ## @param use_service If true, update will be sent via apply service
    def addBox(self, name, size_x, size_y, size_z, x, y, z, use_service=True):
        s = SolidPrimitive()
        s.dimensions = [size_x, size_y, size_z]
        s.type = s.BOX

        ps = PoseStamped()
        ps.header.frame_id = self._fixed_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0

        self.addSolidPrimitive(name, s, ps.pose, use_service)

    ## @brief Attach a box into the planning scene
    ## @param name Name of the object
    ## @param size_x The x-dimensions size of the box
    ## @param size_y The y-dimensions size of the box
    ## @param size_z The z-dimensions size of the box
    ## @param x The x position in link_name frame
    ## @param y The y position in link_name frame
    ## @param z The z position in link_name frame
    ## @param link_name Name of link to attach this object to
    ## @param touch_links Names of robot links that can touch this object
    ## @param use_service If true, update will be sent via apply service
    def attachBox(self, name, size_x, size_y, size_z, x, y, z, link_name,
                  touch_links=None, detach_posture=None, weight=0.0,
                  use_service=True):
        s = SolidPrimitive()
        s.dimensions = [size_x, size_y, size_z]
        s.type = s.BOX

        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1.0
        o = self.makeSolidPrimitive(name, s, p)
        o.header.frame_id = link_name
        a = self.makeAttached(link_name, o, touch_links, detach_posture, weight)
        self._attached_objects[name] = a
        self.sendUpdate(None, a, use_service)

    ## @brief Insert new cube to planning scene
    ## @param use_service If true, update will be sent via apply service
    def addCube(self, name, size, x, y, z, use_service=True):
        self.addBox(name, size, size, size, x, y, z, use_service)

    ## @brief Send message to remove object
    ## @param use_service If true, update will be sent via apply service
    def removeCollisionObject(self, name, use_service=True):
        """ Remove an object. """
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

    ## @brief Send message to remove object
    ## @param use_service If true, update will be sent via apply service
    def removeAttachedObject(self, name, use_service=True):
        """ Remove an attached object. """
        o = AttachedCollisionObject()
        o.object.operation = CollisionObject.REMOVE
        o.object.id = name

        try:
            del self._attached_objects[name]
            self._attached_removed[name] = o
        except KeyError:
            pass

        self.sendUpdate(None, o, use_service)

    ## @brief Update the object lists from a PlanningScene message
    def sceneCb(self, msg, initial=False):
        """ Recieve updates from move_group. """
        self._mutex.acquire()
        for obj in msg.world.collision_objects:
            try:
                if obj.operation == obj.ADD:
                    self._collision.append(obj.id)
                    rospy.logdebug('ObjectManager: Added Collision Obj "%s"',
                                   obj.id)
                    if initial:
                        # this is our initial planning scene, hold onto each object
                        self._objects[obj.id] = obj
                elif obj.operation == obj.REMOVE:
                    self._collision.remove(obj.id)
                    self._removed.pop(obj.id, None)
                    rospy.logdebug('ObjectManager: Removed Collision Obj "%s"',
                                   obj.id)
            except ValueError:
                pass
        self._attached = list()
        for obj in msg.robot_state.attached_collision_objects:
            rospy.logdebug('ObjectManager: attached collision Obj "%s"',
                           obj.object.id)
            self._attached.append(obj.object.id)
            if initial:
                # this is our initial planning scene, hold onto each object
                self._attached_objects[obj.object.id] = obj
        self._mutex.release()

    ## @brief Get a list of names of collision objects
    def getKnownCollisionObjects(self):
        self._mutex.acquire()
        l = copy.deepcopy(self._collision)
        self._mutex.release()
        return l

    ## @brief Get a list of names of attached objects
    def getKnownAttachedObjects(self):
        self._mutex.acquire()
        l = copy.deepcopy(self._attached)
        self._mutex.release()
        return l

    ## @brief Wait for sync
    def waitForSync(self, max_time=2.0):
        sync = False
        t = rospy.Time.now()
        while not sync:
            sync = True
            # delete objects that should be gone
            for name in self._collision:
                if name in self._removed.keys():
                    # should be removed, is not
                    rospy.logwarn('ObjectManager: %s not removed yet', name)
                    self.removeCollisionObject(name, False)
                    sync = False
            for name in self._attached:
                if name in self._attached_removed.keys():
                    # should be removed, is not
                    rospy.logwarn('ObjectManager: Attached object name: %s not removed yet', name)
                    self.removeAttachedObject(name, False)
                    sync = False
            # add missing objects
            for name in self._objects.keys():
                if name not in self._collision + self._attached:
                    rospy.logwarn('ObjectManager: %s not added yet', name)
                    self.sendUpdate(self._objects[name], None, False)
                    sync = False
            for name in self._attached_objects.keys():
                if name not in self._attached:
                    rospy.logwarn('ObjectManager: %s not attached yet', name)
                    self.sendUpdate(None, self._attached_objects[name], False)
                    sync = False
            # timeout
            if rospy.Time.now() - t > rospy.Duration(max_time):
                rospy.logerr('ObjectManager: sync timed out.')
                break
            rospy.logdebug('ObjectManager: waiting for sync.')
            rospy.sleep(0.1)

    ## @brief Set the color of an object
    def setColor(self, name, r, g, b, a=0.9):
        # Create our color
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self._colors[name] = color

    ## @brief Actually send the colors to MoveIt!
    def sendColors(self):
        # Need to send a planning scene diff
        p = PlanningScene()
        p.is_diff = True
        for color in self._colors.values():
            p.object_colors.append(color)
        resp = self._apply_service.call(p)
        if not resp.success:
            rospy.logerr("Could not update colors through service, using topic instead.")
            self._scene_pub.publish(p)
