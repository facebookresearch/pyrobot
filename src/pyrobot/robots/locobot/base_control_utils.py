# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import bezier
import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Twist, PoseStamped
from matplotlib import gridspec
from matplotlib import pyplot as plt
from nav_msgs.srv import GetPlan

from pyrobot.robots.locobot.bicycle_model import wrap_theta, Foo, BicycleSystem

ANGLE_THRESH = 0.05


def build_pose_msg(x, y, theta, frame):
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = frame
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = 0
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]
    return pose_stamped


def _get_absolute_pose(rel_pose, start_pos):
    goal_pos = np.zeros(3)
    x, y, theta = start_pos
    dx, dy, dtheta = rel_pose
    goal_pos[0] = x + dx * np.cos(theta) - dy * np.sin(theta)
    goal_pos[1] = y + dx * np.sin(theta) + dy * np.cos(theta)
    goal_pos[2] = theta + dtheta
    return goal_pos


def get_ramp(ramp, T):
    s = np.linspace(-ramp, ramp, T)
    s = np.tanh(s)
    s = s - np.min(s)
    s = s / np.max(s)
    return s


def linear_interpolate_ramp(start_pos, goal_pos, T, ramp):
    """Linearly interpolates from start to goal, in T steps using a ramp."""
    s = get_ramp(ramp, T)[:, np.newaxis]
    states = start_pos[np.newaxis, :] * (1 - s) + goal_pos[np.newaxis, :] * s
    return states


def pure_rotation_init(start_pos, goal_pos, dt, max_v, max_w, ramp):
    """Function to generate state sequences for pure rotation."""
    # Pure Rotation
    dtheta = goal_pos[2] - start_pos[2]
    dtheta = wrap_theta(dtheta)
    T = int(np.ceil(np.abs(dtheta) / max_w / dt))
    goal1_pos = goal_pos.copy()
    goal1_pos[2] = start_pos[2] + dtheta
    states1 = linear_interpolate_ramp(start_pos, goal1_pos, T, ramp)
    return states1


def sharp_init(start_pos, goal_pos, dt, max_v, max_w, reverse=False):
    f = 0.5
    ramp = 1

    delta_theta = np.arctan2(goal_pos[1] - start_pos[1], goal_pos[0] - start_pos[0])

    to_rotate = wrap_theta(delta_theta - start_pos[2])
    if reverse and np.abs(to_rotate) > 3 * np.pi / 4:
        to_rotate = wrap_theta(np.pi + to_rotate)

    goal1_pos = start_pos.copy()
    if np.abs(to_rotate) > ANGLE_THRESH:
        num_steps_1 = np.ceil(np.abs(to_rotate) / max_w / f / dt).astype(np.int)
        num_steps_1 = np.maximum(num_steps_1, 1)
        goal1_pos[2] = goal1_pos[2] + to_rotate
        states1 = linear_interpolate_ramp(start_pos, goal1_pos, num_steps_1, ramp)
    else:
        states1 = np.zeros([0, 3], dtype=np.float32)

    dist = np.linalg.norm(goal_pos[:2] - start_pos[:2])
    goal2_pos = goal1_pos.copy()
    goal2_pos[:2] = goal_pos[:2]
    num_steps_2 = np.maximum(np.ceil(dist / max_v / f / dt).astype(np.int), 1)
    states2 = linear_interpolate_ramp(goal1_pos, goal2_pos, num_steps_2, ramp)

    to_rotate = wrap_theta(goal_pos[2] - goal2_pos[2])
    goal3_pos = goal2_pos.copy()
    if np.abs(to_rotate) > ANGLE_THRESH:
        num_steps_3 = np.ceil(np.abs(to_rotate) / max_w / f / dt).astype(np.int)
        num_steps_3 = np.maximum(num_steps_3, 1)
        goal3_pos[2] = goal3_pos[2] + to_rotate
        states3 = linear_interpolate_ramp(goal2_pos, goal3_pos, num_steps_3, ramp)
    else:
        states3 = np.zeros([0, 3], dtype=np.float32)
    init_states = np.concatenate([states1, states2, states3], 0)

    return init_states


def position_control_init_fn(
    init_type, start_pos, goal_pos, dt, max_v, max_w, reverse=False
):
    # sharp init, rotates in direction of goal, and then moves to the goal
    # location and then rotates into goal orientation. It takes velcoity limits
    # into account when doing so.

    dist_thresh = 0.01
    dist = np.linalg.norm(goal_pos[:2] - start_pos[:2])
    angle = wrap_theta(goal_pos[2] - start_pos[2])
    already_at_goal = dist < dist_thresh and np.abs(angle) < ANGLE_THRESH
    pure_rotation = dist < dist_thresh
    f = 0.5
    ramp = 1

    if already_at_goal:
        init_states = np.array([start_pos])

    elif pure_rotation:
        init_states = pure_rotation_init(
            start_pos, goal_pos, dt, max_v * f, max_w * f, ramp
        )

    elif not pure_rotation:
        if init_type == "sharp":
            init_states = sharp_init(start_pos, goal_pos, dt, max_v, max_w, reverse)
        elif init_type == "smooth":
            init_states = smooth_init(start_pos, goal_pos, dt, max_v, max_w, reverse)

    return init_states


def smooth_init(start_pos, goal_pos, dt, max_v, max_w, reverse=False):
    # smart init, rotates in direction of goal, and then moves to the goal
    dist = np.linalg.norm(goal_pos[:2] - start_pos[:2])
    # if 5cm from goal, treat it as a pure rotation target.
    pure_rotation = dist < 0.10
    ramp = 1
    f = 0.5
    backward = False
    if pure_rotation:
        init_states = pure_rotation_init(
            start_pos, goal_pos, dt, max_v * f, max_w * f, ramp=ramp
        )
    else:
        s, pts, bezier_curve = bezier_trajectory(
            start_pos,
            goal_pos,
            dt,
            max_v=max_v * f,
            end_derivative_scale=0.8,
            ramp=ramp,
        )
        if reverse:
            s_, pts_, bezier_curve_ = bezier_trajectory(
                start_pos,
                goal_pos,
                dt,
                max_v=max_v * f,
                end_derivative_scale=-0.8,
                ramp=ramp,
            )
            if bezier_curve_.length < bezier_curve.length:
                pts = pts_
                backward = True
        init_states, _ = compute_controls_from_xy(pts, start_pos[2], dt, backward)
    return init_states


def bezier_trajectory(start, end, dt, max_v=0.4, end_derivative_scale=0.5, ramp=3):
    """
    Computes a bezier fit to the start and end conditions, and returns a
    sequence of points that smoothly coneys the robot to the desired target
    configuration.
    """
    x0, y0, th0 = start
    x1, y1, th1 = end
    r = end_derivative_scale
    qs = np.array(
        [
            [x0, y0],
            [x0 + r * np.cos(th0), y0 + r * np.sin(th0)],
            [x1 - r * np.cos(th1), y1 - r * np.sin(th1)],
            [x1, y1],
        ],
        dtype=np.float64,
    )
    bezier_curve = bezier.Curve(qs.T, degree=3)
    T = int(np.ceil(bezier_curve.length / dt / max_v))
    s = get_ramp(ramp, T)
    pts = bezier_curve.evaluate_multi(s).T
    return s, pts, bezier_curve


def compute_controls_from_xy(xy, theta0, dt, flip_theta=False):
    """
    Given the xy trajectory, this computes the orientation, and v and w
    commands to track this trajectory. These can then be used to close the loop
    on this trajectory using an LQR controller.
    """
    x, y = xy.T
    theta = np.arctan2(y[1:] - y[:-1], x[1:] - x[:-1])
    if flip_theta:
        theta = theta + np.pi
    theta = np.concatenate([[theta0], theta], axis=0)
    # Unwrap theta as necessary.
    old_theta = theta[0]
    for i in range(theta.shape[0] - 1):
        theta[i + 1] = wrap_theta(theta[i + 1] - old_theta) + old_theta
        old_theta = theta[i + 1]

    xyt = np.array([x, y, theta]).T
    v = np.linalg.norm(xy[1:, :] - xy[:-1, :], axis=1)
    w = theta[1:] - theta[:-1]
    v = np.append(v, 0)
    w = np.append(w, 0)
    us = np.array([v, w]).T
    us = us / dt
    return xyt, us


class MoveBasePlanner:
    """
    This class enscapsulates the planning capabilities
    of the move_base and provides planning services
    and plan tracking services.
    """

    def __init__(self, configs, action_server):
        self._as = action_server
        self.configs = configs
        self.ROT_VEL = self.configs.MAX_ABS_TURN_SPEED
        self.LIN_VEL = self.configs.MAX_ABS_FWD_SPEED
        self.MAP_FRAME = self.configs.MAP_FRAME
        self.BASE_FRAME = self.configs.VSLAM.VSLAM_BASE_FRAME
        self.point_idx = self.configs.TRACKED_POINT

        rospy.wait_for_service(self.configs.PLAN_TOPIC, timeout=3)
        try:
            self.plan_srv = rospy.ServiceProxy(self.configs.PLAN_TOPIC, GetPlan)
        except rospy.ServiceException:
            rospy.logerr(
                "Timed out waiting for the planning service. \
                    Make sure build_map in script and \
                           use_map in roslauch are set to the same value"
            )
        self.start_state = build_pose_msg(0, 0, 0, self.BASE_FRAME)
        self.tolerance = self.configs.PLAN_TOL
        self._transform_listener = tf.TransformListener()

    def _compute_relative_ang_dist(self, point2):
        # point 1 is the base point 2 is the point on the path
        # convert point 2 to base frame
        pose_stamped = build_pose_msg(point2[0], point2[1], 0, self.MAP_FRAME)
        base_pose = self._transform_listener.transformPose(
            self.BASE_FRAME, pose_stamped
        )

        y = base_pose.pose.position.y
        x = base_pose.pose.position.x

        angle = np.arctan2(y, x)
        distance = np.sqrt(y ** 2 + x ** 2)

        return (angle, distance)

    def get_plan_absolute(self, x, y, theta):
        """
        Gets plan as a list of poses in the world
        frame for the given (x, y, theta
        """
        goal_state = build_pose_msg(x, y, theta, self.MAP_FRAME)
        start_state = self._transform_listener.transformPose(
            self.MAP_FRAME, self.start_state
        )
        response = self.plan_srv(
            start=start_state, goal=goal_state, tolerance=self.tolerance
        )

        status = True
        if len(response.plan.poses) == 0:
            status = False
        return response.plan.poses, status

    def parse_plan(self, plan):
        """Parses the plan generated by move_base service"""
        res_plan = []
        for p in plan:
            p = p.pose
            point = 3 * [0]
            point[0] = p.position.x
            point[1] = p.position.y
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            )
            point[2] = yaw
            res_plan.append(point)
        return res_plan

    def move_to_goal(self, goal, go_to_relative):
        """
        Moves the robot to the robot to given goal state in the absolute frame
        (world frame).

        :param goal: The goal state of the form (x,y,t) in
                     the world (map) frame.
        :param go_to_relative: This is a method that moves the robot the
                               appropiate relative goal while NOT taking map
                               into account.
        :type goal: list
        :type go_to_rative: function of the form foo([x,y,theta])
        """
        plan, plan_status = self.get_plan_absolute(goal[0], goal[1], goal[2])
        if not plan_status:
            rospy.loginfo("Failed to find a valid plan!")
            return False

        if len(plan) < self.point_idx:
            point = goal
        else:
            point = [
                plan[self.point_idx - 1].pose.position.x,
                plan[self.point_idx - 1].pose.position.y,
                0,
            ]

        g_angle, g_distance = self._compute_relative_ang_dist(goal)

        while g_distance > self.configs.TRESHOLD_LIN:

            plan, plan_status = self.get_plan_absolute(goal[0], goal[1], goal[2])

            if self._as.is_preempt_requested():
                return False

            if not plan_status:
                rospy.loginfo("Failed to find a valid plan!")
                return False

            if len(plan) < self.point_idx:
                point = goal
            else:
                point = [
                    plan[self.point_idx - 1].pose.position.x,
                    plan[self.point_idx - 1].pose.position.y,
                    0,
                ]

            angle, distance = self._compute_relative_ang_dist(point)
            g_angle, g_distance = self._compute_relative_ang_dist(goal)
            if not go_to_relative([0, 0, angle]):
                return False
            if not go_to_relative([distance, 0, 0]):
                return False

        g_angle, g_distance = self._compute_relative_ang_dist(goal)

        rospy.loginfo("Adujusting Orientation at goal..{}.".format(g_angle))

        pose_stamped = build_pose_msg(goal[0], goal[1], goal[2], self.MAP_FRAME)
        base_pose = self._transform_listener.transformPose(
            self.BASE_FRAME, pose_stamped
        )

        pose_quat = [
            base_pose.pose.orientation.x,
            base_pose.pose.orientation.y,
            base_pose.pose.orientation.z,
            base_pose.pose.orientation.w,
        ]
        euler = tf.transformations.euler_from_quaternion(pose_quat)
        if not go_to_relative([0, 0, euler[2]]):
            return False


def get_state_trajectory_from_controls(start_pos, dt, controls):
    system = BicycleSystem(dt)
    states = system.unroll(start_pos, controls)
    return states


def get_control_trajectory(trajectory_type, T, v, w):
    if trajectory_type == "circle":
        init_controls = np.zeros((T, 2), dtype=np.float32)
        init_controls[:, 0] = v
        init_controls[:, 1] = w

    elif trajectory_type == "negcircle":
        init_controls = np.zeros((T, 2), dtype=np.float32)
        init_controls[:, 0] = v
        init_controls[:, 1] = -w

    elif trajectory_type == "straight":
        init_controls = np.zeros((T, 2), dtype=np.float32)
        init_controls[:, 0] = v
        init_controls[:, 1] = 0.0

    elif trajectory_type == "rotate":
        init_controls = np.zeros((T, 2), dtype=np.float32)
        init_controls[:, 0] = 0.0
        init_controls[:, 1] = w

    elif trajectory_type == "negrotate":
        init_controls = np.zeros((T, 2), dtype=np.float32)
        init_controls[:, 0] = 0.0
        init_controls[:, 1] = -w

    else:
        raise ValueError("Unknown type: %s" % trajectory_type)

    return init_controls


def get_trajectory_circle(start_pos, dt, r, v, angle):
    w = v / r
    T = int(np.round(angle / w / dt))
    controls = get_control_trajectory("circle", T, v, w)
    states = get_state_trajectory_from_controls(start_pos, dt, controls)
    return states, controls


def get_trajectory_negcircle(start_pos, dt, r, v, angle):
    w = v / r
    T = int(np.round(angle / w / dt))
    controls = get_control_trajectory("circle", T, v, -w)
    states = get_state_trajectory_from_controls(start_pos, dt, controls)
    return states, controls

class LQRSolver(object):
    """
    This class implements a solver for a time-varying Linear Quadratic
    Regulator System. A time-varying LQR system is defined via affine time
    varying transition functions, and time-varying quadratic cost
    functions.Such a system can be solved using dynamic programming to obtain
    time-varying feedback control matrices that can be used to compute the
    control command given the state of the system at any given time step.
    """

    def __init__(
        self, As=None, Bs=None, Cs=None, Qs=None, Rs=None, x_refs=None, u_refs=None
    ):
        """
        Constructor for LQR solver.

        System definition: :math:`x_{t+1} = A_tx_t + B_tu_t + C_t`

        State cost: :math:`x^t_tQ_tx_t + x^t_tq + q\_`

        Control cost: :math:`u^t_tR_tu_t`

        :param As: List of time-varying matrices A_t for dynamics
        :param Bs: List of time-varying matrices B_t for dynamics
        :param Cs: List of time-varying matrices C_t for dynamics
        :param Qs: List of time-varying matrices Q_t for state cost
        :param Rs: List of time-varying matrices R_t for control cost
        :type As: List of state_dim x state_dim numpy matrices
        :type Bs: List of state_dim x control_dim numpy matrices
        :type Cs: List of state_dim x 1 numpy matrices
        :type Qs: List of 3 tuples with numpy array of size state_dim x
                  state_dim, state_dim x 1, 1 x 1
        :type Rs: List of control_dim x control_dim numpy matrices
        """
        if As is None:
            As = []
        if Bs is None:
            Bs = []
        if Cs is None:
            Cs = []
        if Qs is None:
            Qs = []
        if Rs is None:
            Rs = []
        if x_refs is None:
            x_refs = []
        if u_refs is None:
            u_refs = []
        assert len(As) == len(Bs)
        assert len(As) == len(Cs)
        assert len(As) == len(Qs)
        assert len(As) == len(Rs)
        self.As = As
        self.Bs = Bs
        self.Cs = Cs
        self.Qs = Qs
        self.Rs = Rs
        self.x_refs = x_refs
        self.u_refs = u_refs
        self.T = len(As)

    def solve(self):
        """
        Solves the LQR system and stores the solution, such that it can be
        accessed using get_control() function.
        """
        Qs = self.Qs
        As = self.As
        Bs = self.Bs
        Rs = self.Rs
        Cs = self.Cs
        T = len(Qs)
        Ps = [None for _ in range(T)]
        Ks = [None for _ in range(T)]
        u_dim = Bs[0].shape[1]
        x_dim = Bs[0].shape[0]
        for i in range(T):
            # Computing things for i steps to go.
            if i == 0:
                Ps[T - 1 - i] = Qs[T - 1 - i]
                Ks[T - 1 - i] = (
                    np.zeros((u_dim, x_dim), dtype=np.float64),
                    np.zeros((u_dim, 1)),
                )
            else:
                Ks[T - 1 - i], Ps[T - 1 - i] = self._one_step(
                    As[T - 1 - i],
                    Bs[T - 1 - i],
                    Cs[T - 1 - i],
                    Qs[T - 1 - i],
                    Rs[T - 1 - i],
                    Ps[T - i],
                )
        self.Ks = Ks
        self.Ps = Ps

    def _one_step(self, A, B, C, Q_q_q_, R, P_p_p_):
        """
        Executes one step of dynamic programming. Given cost to go for i steps,
        computes cost to go for i+1 steps.
        """
        Q, q, q_ = Q_q_q_
        P, p, p_ = P_p_p_
        # state cost is xtQx + xtq + q_
        # control cost is utRu
        # dynamics are x_{t+1} = A*x_t + B*u_t + C
        # Cost to go is (from previous time step):
        #   xtPx + xtp + p_
        # Cost to go for current time step is of the form:
        #   xt_Ax + ut_Bu + xt_Cu + xt_D + ut_E + _F
        _A = Q + np.dot(np.dot(A.T, P), A)
        _B = R + np.dot(np.dot(B.T, P), B)
        _C = 2 * np.dot(np.dot(A.T, P), B)
        _D = q + np.dot(A.T, p) + 2 * np.dot(np.dot(A.T, P), C)
        _E = np.dot(B.T, p) + 2 * np.dot(np.dot(B.T, P), C)
        _F = p_ + q_ + np.dot(np.dot(C.T, P), C) + np.dot(p.T, C)
        inv_B = np.linalg.inv(_B)
        k = -0.5 * np.dot(inv_B, _C.T)
        k_ = -0.5 * np.dot(inv_B, _E)

        o_P = _A - 0.25 * (np.dot(np.dot(_C, inv_B.T), _C.T))
        o_p = _D - 0.5 * (np.dot(np.dot(_C, inv_B.T), _E))
        o_p_ = _F - 0.25 * (np.dot(np.dot(_E.T, inv_B.T), _E))

        return (k, k_), (o_P, o_p, o_p_)

    def get_control(self, x, i):
        """
        Uses the stored solution to the system to output control cost if the
        system is in state x at time step i.

        :param x: state of the system
        :param i: time step
        :type x: numpy array (state_dim,)
        :type i: int

        :return: feedback control that should be applied
        :rtype: numpy array (control_dim,)
        """
        u = np.dot(self.Ks[i][0], x[:, np.newaxis]) + self.Ks[i][1]
        u = u[:, 0]
        return u

    def get_control_ls(self, x, alpha, i):
        """
        Returns control but modulated via a step-size alpha.

        :param x: state of the system
        :param alpha: step size
        :param i: time step
        :type x: numpy array (state_dim,)
        :type alpha: float
        :type i: int

        :return: feedback control that should be applied
        :rtype: numpy array (control_dim,)
        """
        x = x[:, np.newaxis]
        u = (
            np.dot(self.Ks[i][0], (x - self.x_refs[i]))
            + alpha
            * (self.Ks[i][1] - self.u_refs[i] + np.dot(self.Ks[i][0], self.x_refs[i]))
            + self.u_refs[i]
        )
        u = u[:, 0]
        return u

    def get_cost_to_go(self, x, i):
        """
        Returns cost to go if the system is in state x at time step i.

        :param x: state of the system
        :param i: time step
        :type x: numpy array (state_dim,)
        :type i: int

        :return: cost if system were to follow optimal f
                 eedback control from now
        :rtype: scalar
        """
        x = x[:, np.newaxis]
        c = (
            np.dot(np.dot(x.T, self.Ps[i][0]), x)
            + np.dot(x.T, self.Ps[i][1])
            + self.Ps[i][2]
        )
        return c[0, 0]


class ILQRSolver(object):
    """
    Iterative LQR solver for non-linear systems. Computes a linearization of
    the system at the current trajectory, and solves that linear system using
    LQR. This process is repeated.
    """

    def __init__(self, dyn_fn, Q_fn, R_fn, start_state, goal_state):
        """
        Constructor that sets up functions describing the system and costs.

        :param Q_fn: State cost function that takes as input x_goal, x_ref,
                     time_step, lqr_iteration, and returns quadratic
                     approximations of state cost around x_ref.
        :param R_fn: Control cost function that takes as input u_ref and
                     returns quadtratic approximation around it.
        :param dyn_fn: Dynamics function thattakes in state, controls,
                       return_only_state flag, and returns the linearization
                       and the next state.
        :param start_state: Starting state of the system.
        :param goal_state: Goal state of the system.
        :type Q_fn: python function
        :type R_fn: python function
        :type dyn_fn: python function
        :type start_state: numpy vector
        :type goal_state: numpy vector
        """
        self.dyn_fn = dyn_fn
        self.Q_fn = Q_fn
        self.R_fn = R_fn
        self.start_state = start_state
        self.goal_state = goal_state

    def unroll(self, dyn_fn, start_state, controls):
        """
        Obtain state trajectory by applying controls on the system defined by
        the dynamics function dyn_fn.

        :param Q_fn: State cost function that takes as input x_goal, x_ref,
                     time_step, lqr_iteration, and returns quadratic
                     approximations of state cost around x_ref.
        :param start_state: Starting state of the system.
        :param controls: Sequence of controls to apply to the system.
        :type Q_fn: python function
        :type start_state: numpy vector
        :type controls: numpy array

        :return: Sequence of states
        :rtype: numpy array
        """
        T = len(controls)
        states = []
        state = start_state.copy()
        for j in range(T):
            states.append(state)
            _, _, _, state = dyn_fn(state, controls[j], True)
        return states

    def solve(self, init_controls, ilqr_iters):
        """
        Solve the non-linear system.

        :param init_controls: Initial control sequence to start optimization
                              from.
        :param ilqr_iters: Number of iterations of linearizations and LQR
                           solutions.
        :type init_controls: numpy array
        :param ilqr_iters: int

        :return: LQR Tracker, step_size, cost of solution
        :rtype: LQRSolver, int, cost
        """
        controls = init_controls
        T = len(controls)

        total_costs = []
        for i in range(ilqr_iters):
            # Unroll trajectory.
            states = self.unroll(self.dyn_fn, self.start_state, controls)

            # Linearize around trajectory.
            As = []
            Bs = []
            Cs = []
            Qs = []
            Rs = []
            total_cost_i = 0.0
            for j in range(T):
                A, B, C, _ = self.dyn_fn(states[j], controls[j], False)
                As.append(A)
                Bs.append(B)
                Cs.append(C)
                Q, q, q_, x_cost = self.Q_fn(self.goal_state, states[j], j, i)
                Qs.append((Q, q, q_))
                R, r, r_, u_cost_ = self.R_fn(controls[j])
                u_cost = np.dot(np.dot(controls[j][:, 0].T, R), controls[j][:, 0])
                Rs.append(R)
                total_cost_i += x_cost + u_cost
            total_costs.append(total_cost_i)

            lqr_ = LQRSolver(As, Bs, Cs, Qs, Rs, states, controls)
            lqr_.solve()

            step_size, controls, cost = self.get_step_size(
                lqr_, controls, total_cost_i, i
            )
            if step_size == 0:
                break
        return lqr_, step_size, cost

    def get_step_size(self, lqr_, ref_controls, ref_cost, ilqr_iter):
        """
        Search for the step size that improves the cost function over LQR
        iterations.

        :param ilqr_iter: Which ILQR iteration are we doing so as to compute
                          the cost function which may depend on the
                          ilqr_iteration for a log barrier method.
        :param ref_controls: Reference controls, we are starting iterations
                             from.
        :param ref_cost: Refernce cost that we want to improve over.
        :type ilqr_iter: int
        :type ref_cost: float
        :type ref_controls: numpy array

        :return: step size, updated controls, updated cost
        :rtype: float, numpy array, float
        """
        out_controls = ref_controls
        out_step_size = 0.0
        out_cost = ref_cost

        step_size = 1.0
        while step_size > 0.00001:
            controls_ = []
            state = self.start_state.copy()
            q_line_search = 0.0
            T = len(ref_controls)
            for j in range(T):
                u = lqr_.get_control_ls(state, step_size, j)
                R, r, r_, u_cost = self.R_fn(u)
                _, _, _, x_cost = self.Q_fn(self.goal_state, state, j, ilqr_iter)
                controls_.append(u)
                _, _, _, state = self.dyn_fn(state, controls_[j], True)
                q_line_search += x_cost + u_cost

            if q_line_search < ref_cost:
                out_controls = controls_
                out_step_size = step_size
                out_cost = q_line_search
                break
            step_size = step_size * 0.5
        return out_step_size, out_controls, out_cost


class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2


def check_server_client_link(client):
    rospy.sleep(0.1)  # Ensures client spins up properly
    server_up = client.wait_for_server(timeout=rospy.Duration(10.0))
    if not server_up:
        rospy.logerr(
            "Timed out waiting for the client"
            " Action Server to connect. Start the action server"
            " before running example."
        )
        rospy.signal_shutdown("Timed out waiting for Action Server")
        sys.exit(1)


class LocalActionStatus:
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    FREE = 5
    UNKOWN = 6
    PREEMPTING = 7
    DISABLED = 8


import threading
import copy


class LocalActionServer(object):
    """docstring for LocalActionServer"""

    def __init__(self):

        self._lock = threading.RLock()
        self._state = LocalActionStatus.UNKOWN

    def is_preempt_requested(self):
        if (
            self.get_state() == LocalActionStatus.PREEMPTING
            or self.get_state() == LocalActionStatus.DISABLED
        ):
            return True
        else:
            return False

    def get_state(self):
        self._lock.acquire()
        state = copy.copy(self._state)
        self._lock.release()
        return state

    def _set_state(self, state):
        self._lock.acquire()
        self._state = state
        self._lock.release()

    def cancel_goal(self):
        self._set_state(LocalActionStatus.PREEMPTING)

    def set_preempted(self):
        self._set_state(LocalActionStatus.PREEMPTED)

    def set_succeeded(self):
        self._set_state(LocalActionStatus.SUCCEEDED)

    def set_aborted(self):
        self._set_state(LocalActionStatus.ABORTED)

    def set_active(self):
        self._set_state(LocalActionStatus.ACTIVE)

    def is_disabled(self):
        state = self.get_state()
        if state == LocalActionStatus.DISABLED:
            return True
        return False

    def disable(self):
        self._set_state(LocalActionStatus.DISABLED)

    def is_available(self):
        state = self.get_state()
        if (
            state == LocalActionStatus.ACTIVE
            or state == LocalActionStatus.PREEMPTING
            or state == LocalActionStatus.DISABLED
        ):
            return False
        else:
            return True
