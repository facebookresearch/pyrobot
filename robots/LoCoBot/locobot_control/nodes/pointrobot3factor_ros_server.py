#!/usr/bin/env python

#!/usr/bin/env python


import numpy as np
from gtsam import *
from gpmp2 import *

import threading
import copy
import actionlib
import sys
import math
from scipy import ndimage

import rospy
import tf
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, OccupancyGrid

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectoryPoint


# TOPIC NAMES
odom_topic = "/odom"
cmd_vel_topic = "/cmd_vel_mux/input/navi"
action_topic = "/gpmp_ctrl"
occ_grid_topic = "/move_base/local_costmap/costmap"


class RobotState(object):
    """A simple robot state object to keep track of locobot's base state"""

    def __init__(self):
        self.pose = None
        self.vel = None


class Robot(object):
    """

    A simple locobot interface object which is PyRobot independent.
    Functions:
    Subscribes to robot state,
    Subscribe to occupancy grid and builds GPMP SDF
    Publishes gpmp trajectory to the trajectory action server.

    """

    def __init__(self):

        self.ctrl_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.state = RobotState()
        self.sdf = None
        self.sdf_lock = threading.RLock()

        rospy.Subscriber(odom_topic, Odometry, self._odometry_callback)

        rospy.Subscriber(occ_grid_topic, OccupancyGrid, self._occ_grid_callback)

        self.traj_client_ = actionlib.SimpleActionClient(
            "/turtle/base_controller/trajectory", FollowJointTrajectoryAction,
        )

        server_up = self.traj_client_.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr(
                "Timed out waiting for Joint Trajectory"
                " Action Server to connect. Start the action server"
                " before running example."
            )
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

    def _occ_grid_callback(self, msg):
        cols = msg.info.width
        rows = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        cell_size = msg.info.resolution
        occ_map = np.zeros((rows, cols))

        for i in range(rows):
            for j in range(cols):
                k = i * cols + j
                if msg.data[k] > 0:
                    occ_map[i][j] = 1
                else:
                    occ_map[i][j] = 0

        # Signed Distance field
        origin_point2 = Point2(origin_x, origin_y)
        field = signedDistanceField2D(occ_map, cell_size)
        sdf = PlanarSDF(origin_point2, cell_size, field)

        self.sdf_lock.acquire()
        self.sdf = sdf
        self.sdf_lock.release()

    def _odometry_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        x_vel = msg.twist.twist.linear.x
        y_vel = 0.0
        ang_vel = msg.twist.twist.angular.z
        self.state.pose = np.asarray([x, y, yaw])
        self.state.vel = np.asarray([x_vel, y_vel, ang_vel])

    def stop(self):
        rospy.loginfo("Stopping base!")
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.ctrl_pub.publish(msg)

    def get_robot_state(self):
        return self.state.pose, self.state.vel

    def get_local_frame_vels(self, temp_vel,  yaw):
        # temp_vel is velocity in global frame.

        vel = np.asarray([1.0,1.0,1.0])

        vel[0] = temp_vel[0] * np.cos(yaw) + temp_vel[1]* np.sin(yaw)
        vel[1] = -1.0 * temp_vel[0] * np.sin(yaw) +  temp_vel[1] * np.cos(yaw)
        vel[2] = temp_vel[2]

        return vel
    
    
    def executeTrajectory(self, result, params):

        DOF = 3
        traj_ = FollowJointTrajectoryGoal()
        point = JointTrajectoryPoint()

        for i in range(params.total_time_step):

            pose = result.atVector(symbol(ord("x"), i))
            vel = self.get_local_frame_vels(result.atVector(symbol(ord("v"), i)), 
                                            pose[2])

            point = JointTrajectoryPoint()

            for j in range(3):
                point.positions.append(pose[j])
                point.velocities.append(vel[j])

            point.time_from_start = rospy.Duration(
                i * params.delta_t
            )  # /(check_inter+1))
            traj_.trajectory.points.append(point)

            traj_.trajectory.header.stamp = rospy.Time.now()

        self.traj_client_.cancel_goal()
        self.traj_client_.send_goal(traj_)
        # self.traj_client_.wait_for_result()


def signedDistanceField2D(ground_truth_map, cell_size):
    # SIGNEDDISTANCEFIELD2D 2D signed distance field
    #   Given a ground truth 2D map defined in Matrix in 0-1,
    #   calculate 2D signed distance field, which is defined as a matrix
    #   map matrix and signed distance field matrix have the same resolution.
    #
    #   Usage: field = SIGNEDDISTANCEFIELD2D(ground_truth_map, cell_siz)
    #   @map        evidence grid from dataset, map use 0 show open area, 1 show objects.
    #   @cell_size  cell sizeto given metric information
    #
    #   Output:
    #   @field      sdf, row is Y, col is X

    # regularize unknow area to open area
    cur_map = ground_truth_map > 0.75
    cur_map = cur_map.astype(int)

    if np.amax(cur_map) is 0:
        return np.ones(ground_truth_map.shape) * 1000

    # inverse map
    inv_map = 1 - cur_map

    # get signed distance from map and inverse map
    # since bwdist(foo) = ndimage.distance_transform_edt(1-foo)
    map_dist = ndimage.distance_transform_edt(inv_map)
    inv_map_dist = ndimage.distance_transform_edt(cur_map)

    field = map_dist - inv_map_dist

    # metric
    field = field * cell_size
    field = field.astype(float)

    return field


def get_plan(start_conf_val, start_vel, end_conf_val, end_vel, sdf, params):

    start_conf = start_conf_val
    end_conf = end_conf_val

    avg_vel = (end_conf_val - start_conf_val / params.total_time_step) / params.delta_t

    # plot param

    # init optimization
    graph = NonlinearFactorGraph()
    init_values = Values()

    for i in range(0, params.total_time_step + 1):
        key_pos = symbol(ord("x"), i)
        key_vel = symbol(ord("v"), i)

        #% initialize as straight line in conf space
        pose = start_conf_val
        vel = avg_vel

        init_values.insert(key_pos, pose)
        init_values.insert(key_vel, vel)

        #% start/end priors
        if i == 0:
            graph.push_back(PriorFactorVector(key_pos, start_conf, params.pose_fix))
            graph.push_back(PriorFactorVector(key_vel, start_vel, params.vel_fix))
        elif i == params.total_time_step:
            graph.push_back(PriorFactorVector(key_pos, end_conf, params.pose_fix_goal))
            graph.push_back(PriorFactorVector(key_vel, end_vel, params.vel_fix_goal))

        graph.add(VehicleDynamicsFactorVector(key_pos, key_vel, params.cost_sigma))

        # GP priors and cost factor
        if i > 0:
            # graph.push_back(PriorFactorVector(key_pos, end_conf, params.pose_fix_goal))
            # graph.push_back(PriorFactorVector(key_vel, end_vel, params.vel_fix_goal))
            key_pos1 = symbol(ord("x"), i - 1)
            key_pos2 = symbol(ord("x"), i)
            key_vel1 = symbol(ord("v"), i - 1)
            key_vel2 = symbol(ord("v"), i)

            temp = GaussianProcessPriorLinear(
                key_pos1, key_vel1, key_pos2, key_vel2, params.delta_t, params.Qc_model
            )
            graph.push_back(temp)

            #% cost factor
            graph.push_back(
                ObstaclePlanarSDFFactorPointRobot(
                    key_pos,
                    params.pR_model,
                    sdf,
                    params.cost_sigma,
                    params.epsilon_dist,
                )
            )

            #% GP cost factor
            if params.use_GP_inter and params.check_inter > 0:
                for j in range(1, params.check_inter + 1):
                    tau = j * (params.total_time_sec / params.total_check_step)
                    graph.add(
                        ObstaclePlanarSDFFactorGPPointRobot(
                            key_pos1,
                            key_vel1,
                            key_pos2,
                            key_vel2,
                            params.pR_model,
                            sdf,
                            params.cost_sigma,
                            params.epsilon_dist,
                            params.Qc_model,
                            params.delta_t,
                            tau,
                        )
                    )

    if params.use_trustregion_opt:
        parameters = DoglegParams()
        optimizer = DoglegOptimizer(graph, init_values, parameters)
    else:
        parameters = GaussNewtonParams()
        optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

    print("Initial Error = %d\n", graph.error(init_values))

    optimizer.optimizeSafely()
    result = optimizer.values()

    print("Final Error = %d\n", graph.error(result))

    res_flag = True
    if graph.error(result) > params.acceptable_error_threshold:
        res_flag = False
    return result, res_flag


class Parameters(object):  # TODO: read from yaml file or rosparams
    # settings
    total_time_sec = 5.0
    total_time_step = 50
    total_check_step = 50.0
    delta_t = total_time_sec / total_time_step
    check_inter = int(total_check_step / total_time_step - 1)

    use_GP_inter = True

    # point robot model
    pR = PointRobot(3, 1)
    spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1.5])
    nr_body = spheres_data.shape[0]
    sphere_vec = BodySphereVector()
    sphere_vec.push_back(
        BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
    )
    pR_model = PointRobotModel(pR, sphere_vec)

    # GP
    Qc = np.identity(pR_model.dof())
    Qc_model = noiseModel_Gaussian.Covariance(Qc)

    # Obstacle avoid settings
    cost_sigma = 0.005
    epsilon_dist = 0.1

    # prior to start/goal
    pose_fix = pose_fix_goal = noiseModel_Isotropic.Sigma(pR_model.dof(), 0.0001)
    vel_fix = vel_fix_goal = noiseModel_Isotropic.Sigma(pR_model.dof(), 0.0001)

    use_trustregion_opt = True

    pause_time = total_time_sec / total_time_step

    # Fixed window params
    goal_region_threshold = 0.1
    acceptable_error_threshold = 400
    sigma_goal = 4

    opt_timeout = 0.2


class GPMPController(object):
    """docstring for GPMPController"""

    def __init__(self, robot, params, action_name):

        self.robot = robot
        self.params = params
        self._action_name = action_name
        # Action server for the pyrobot client

        self._as = actionlib.SimpleActionServer(
            self._action_name,
            FollowJointTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()

    def execute_cb(self, goal):

        # start and end conf
        end_conf_val = np.asarray(goal.trajectory.points[0].positions)
        end_vel = np.asarray(goal.trajectory.points[0].velocities)
        goal_region_threshold = goal.goal_tolerance[0].position
        duration = goal.goal_time_tolerance.to_sec()

        print("Received goal", end_conf_val, end_vel)
        start_time = rospy.get_time()
        curstate_val, curstate_vel = self.robot.get_robot_state()
        init_distance = np.linalg.norm(curstate_val - end_conf_val)
        while np.linalg.norm(curstate_val - end_conf_val) > goal_region_threshold:

            # Timeout
            if rospy.get_time() - start_time > duration:
                rospy.logerr(
                    "The controller timedout trying to reach the goal."
                    " Consider increasing the time"
                )
                self.robot.traj_client_.cancel_goal()
                self.robot.stop()
                self._as.set_aborted()
                return

            if self._as.is_preempt_requested():
                rospy.logwarn(
                    "##############   %s: Preempted ####################"
                    % self._action_name
                )
                self._as.set_preempted()
                # Note: The trajectory is not cancelled for preempt as updated trajectory would be given
                return

            # Goal prior factors
            self.params.pose_fix_goal = noiseModel_Isotropic.Sigma(
                3,
                self.params.sigma_goal
                * np.linalg.norm(curstate_val - end_conf_val)
                / init_distance,
            )
            self.params.vel_fix_goal = noiseModel_Isotropic.Sigma(
                3,
                self.params.sigma_goal
                * np.linalg.norm(curstate_val - end_conf_val)
                / init_distance,
            )

            self.robot.sdf_lock.acquire()
            sdf = self.robot.sdf
            self.robot.sdf_lock.release()

            result, res_flag = get_plan(
                curstate_val, curstate_vel, end_conf_val, end_vel, sdf, self.params
            )

            if not res_flag:
                rospy.logerr("GPMP optimizer failed to produce an acceptable plan")
                self.robot.traj_client_.cancel_goal()
                self.robot.stop()
                self._as.set_aborted()
                return

            self.robot.executeTrajectory(result, self.params)
            rospy.sleep(0.5)

            curstate_val, curstate_vel = self.robot.get_robot_state()
            print("Current State: ", curstate_val, curstate_vel)
            print("Error", np.linalg.norm(curstate_val - end_conf_val))

        self.robot.traj_client_.wait_for_result()  # TODO: Absorb this into the treshold

        self._as.set_succeeded()


def main():

    try:
        rospy.init_node("gpmp_controller_server", anonymous=True)
    except rospy.exceptions.ROSException:
        rospy.logwarn("ROS node [gpmp_controller] has already been initialized")

    robot = Robot()
    while robot.sdf is None:
        rospy.logwarn("Waiting for robot SDF!!")
        rospy.sleep(0.2)
    params = Parameters()

    gpmp_controller = GPMPController(robot, params, "/gpmp_controller")

    rospy.spin()


if __name__ == "__main__":
    main()
