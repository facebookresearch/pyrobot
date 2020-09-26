from pyrobot.algorithms.base_controller import BaseController
from pyrobot.algorithms.base_localizer import BaseLocalizer

from pyrobot.algorithms.base_controller_impl.ilqr_utils import Foo, BicycleSystem

from pyrobot.algorithms.base_controller_impl.base_control_utils import LQRSolver, _get_absolute_pose, position_control_init_fn


from geometry_msgs.msg import Twist

import rospy
import copy

import numpy as np


class ILQRControl(BaseController):
    """base class of Motion Planning algorithms.
    Specifically, forward/inverse kinematics, and jacobian.
    """

    def __init__(
        self,
        configs,
        world,
        ros_launch_manager=None,
        robots={},
        sensors={},
        algorithms={},
    ):
        super(BaseController, self).__init__(
            configs,
            world,
            ros_launch_manager,
            robots,
            sensors,
            algorithms,
        )

        self.robot_label = list(self.robots.keys())[0]
        self.bot_base = self.robots[self.robot_label]["base"]

        self._as = self.bot_base._as

        self.ctrl_pub = self.bot_base.ctrl_pub

        self.max_v = self.configs.MAX_ABS_FWD_SPEED
        self.min_v = -self.configs.MAX_ABS_FWD_SPEED
        self.max_w = self.configs.MAX_ABS_TURN_SPEED
        self.min_w = -self.configs.MAX_ABS_TURN_SPEED
        self.rate = rospy.Rate(self.configs.BASE_CONTROL_RATE)
        self.dt = 1.0 / self.configs.BASE_CONTROL_RATE

        self.system = BicycleSystem(
            self.dt, self.min_v, self.max_v, self.min_w, self.max_w
        )

    def go_to_relative(self, xyt_position, close_loop=True, smooth=True):
        start_pos = self.algorithms["BaseLocalizer"].get_odom_state()
        goal_pos = _get_absolute_pose(xyt_position, start_pos.ravel())
        return self.go_to_absolute(goal_pos, close_loop, smooth)

    def go_to_absolute(self, xyt_position, close_loop=True, smooth=True):
        start_pos = self.algorithms["BaseLocalizer"].get_odom_state()
        goal_pos = copy.deepcopy(xyt_position)
        reverse = self.min_v < 0
        states = self._compute_trajectory_no_map(start_pos, goal_pos, smooth, reverse)
        # Compute and execute the plan.
        return self.track_trajectory(states, close_loop=close_loop)

    def check_cfg(self):
        assert len(self.robots.keys()) == 1, "One Localizer only handle one base!"
        robot_label = list(self.robots.keys())[0]
        assert (
            "base" in self.robots[robot_label].keys()
        ), "base required for base controllers!"

        assert (
            len(self.algorithms.keys()) == 1
        ), "ilqr controller only have one dependency!"
        assert (
            list(self.algorithms.keys())[0] == "BaseLocalizer"
        ), "ilqr controller only depend on BaseLocalizer!"
        assert isinstance(
            self.algorithms["BaseLocalizer"], BaseLocalizer
        ), "BaseLocalizer module needs to extend BaseLocalizer base class!"

        assert "MAX_ABS_FWD_SPEED" in self.configs.keys()
        assert "MAX_ABS_TURN_SPEED" in self.configs.keys()
        assert "BASE_CONTROL_RATE" in self.configs.keys()

    def _compute_trajectory_no_map(self, start_pos, goal_pos, smooth, reverse):
        typ = "smooth" if smooth else "sharp"
        init_states = position_control_init_fn(
            typ, start_pos, goal_pos, self.dt, self.max_v, self.max_w, reverse
        )
        return init_states

    def track_trajectory(self, states, close_loop=True):
        """
        State trajectory that the robot should track using ILQR control.

        :param states: sequence of (x,y,t) states that the robot should track.
        :param controls: optionally specify control sequence as well.
        :param close_loop: whether to close loop on the
                           computed control sequence or not.

        :type states: list
        :type controls: list
        :type close_loop: bool
        """
        # Compute plan
        plan = self._generate_plan(states)
        if not plan:
            return False
        # Execute a plan
        return self._execute_plan(plan, close_loop)

    def _generate_plan(self, xs):
        """Generates a feedback controller that tracks the state trajectory
        specified by xs. Optionally specify a control trajectory us, otherwise
        it is automatically inferred.

        :param xs: List of states that define the trajectory.
        :param us: List of controls that define the trajectory. If None,
                   controls are inferred from the state trajectory.

        :returns: LQR controller that can track the specified trajectory.
        :rtype: LQRSolver
        """

        iters = 2
    
        # Initialize with simple non-saturated controls.
        us = np.zeros((len(xs), 2), dtype=np.float32) + 0.1

        for i in range(iters):
            T = len(us)
            As = []
            Bs = []
            Cs = []
            Qs = []
            Rs = []
            x_costs = []
            u_costs = []
            total_cost = 0.0
            controls = us
            states = xs
            for j in range(T):

                if self._as.is_preempt_requested():
                    rospy.loginfo(
                        "Preempted the ILQR execution. Plan generation failed"
                    )
                    return False

                A, B, C, _ = self.system.dynamics_fn(states[j], controls[j])
                As.append(A)
                Bs.append(B)
                Cs.append(C)
                Q, q, q_, x_cost = self.system.get_system_cost(xs[j], states[j])
                Qs.append((Q, q, q_))
                R, r, r_, u_cost_ = self.system.get_control_cost(controls[j])
                u_cost = np.dot(
                    np.dot(controls[j][:, np.newaxis].T, R), controls[j][:, np.newaxis]
                )
                Rs.append(R)
                total_cost += x_cost + u_cost
                x_costs.append(x_cost)
                u_costs.append(u_cost)
            plan = LQRSolver(As, Bs, Cs, Qs, Rs, states, controls)
            plan.solve()
            if i < iters - 1:
                # We will do more iterations
                us = self._compute_controls(states[0], plan)
        return plan

    def _compute_controls(self, start_state, plan):
        state = start_state
        us = []
        for j in range(plan.T):
            u = plan.get_control(state, j)
            _, _, _, state = self.system.dynamics_fn(state, u)
            us.append(u)
        us = np.array(us)
        return us

    def _execute_plan(self, plan, close_loop=True):
        """
        Executes the plan, check for conditions like bumps, etc.
        Plan is object returned from generate_plan.
        Also stores the plan execution into
        variable self._trajectory_tracker_execution.

        :param plan: Object returned from generate_plan function.
        :param close_loop: Whether to use feedback controller, or apply
                           open-loop commands.
        :type plan: LQRSolver
        :type close_loop: bool

        :returns: Weather plan execution was successful or not.
        :rtype: bool
        """
        twist_min = np.array([self.min_v, self.min_w], dtype=np.float32)
        twist_max = np.array([self.max_v, self.max_w], dtype=np.float32)
        twist = Twist()
        success = True
        xs = []
        xrefs = []
        us = []
        urefs = []
        for i in range(plan.T):

            if self._as.is_preempt_requested():
                rospy.loginfo("Preempted the ILQR execution. Execution failed")
                return False

            if self.bot_base.base_state.should_stop:
                self.stop()
                self.bot_base.base_state.should_stop = False
                return False

            # Apply control for this time stpe.
            # Read the current state of the system and apply control.
            x = self.algorithms["BaseLocalizer"].get_odom_state()
            u = plan.get_control(x, i)
            u_ref = plan.u_refs[i]
            if close_loop:
                u_apply = u.copy()
            else:
                u_apply = u_ref.copy()
            u_apply = np.clip(u_apply, a_min=twist_min, a_max=twist_max)
            twist.linear.x = u_apply[0]
            twist.angular.z = u_apply[1]
            self.ctrl_pub.publish(twist)

            xs.append(x)
            us.append(u_apply)
            xrefs.append(plan.x_refs[i])
            urefs.append(plan.u_refs[i])
            # wait for 0.1 seconds (10 HZ) and publish again
            self.rate.sleep()
        xs = np.array(xs).reshape((-1, 3))
        xrefs = np.array(xrefs).reshape((-1, 3))
        urefs = np.array(urefs).reshape((-1, 2))
        us = np.array(us).reshape((-1, 2))
        xus = np.concatenate((xs, us), 1)
        xurefs = np.concatenate((xrefs, urefs), 1)
        self._trajectory_tracker_execution = Foo(xus=xus, xurefs=xurefs)
        return success

    def stop(self):
        """
        Stops the base.
        """
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.ctrl_pub.publish(msg)
