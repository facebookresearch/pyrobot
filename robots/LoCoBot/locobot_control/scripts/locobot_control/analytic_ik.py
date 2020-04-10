# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# for calculating inverse kinematics of 5DOF robot

# mechanism
#
#    /\
# l2 /  \l3
#  /    \___
#  |      l4(alpha-oreintation of this link with ground)
#  |l1(fixed)
#  |
# -----
# co-ordinate axis
# X-->
# ^
# |
# Z

import copy
import math
from math import pi

import rospy
import tf


class AnalyticInverseKinematics:
    def __init__(self, base_frame="/base_link", tool_frame="/gripper_link"):
        # robot parmas
        listener = tf.TransformListener()
        listener.waitForTransform(
            base_frame, "/arm_base_link", rospy.Time(), rospy.Duration(2.0)
        )
        listener.waitForTransform(
            "/arm_base_link", "/elbow_link", rospy.Time(), rospy.Duration(2.0)
        )
        listener.waitForTransform(
            "/elbow_link", "/forearm_link", rospy.Time(), rospy.Duration(2.0)
        )
        listener.waitForTransform(
            "/forearm_link", "/wrist_link", rospy.Time(), rospy.Duration(2.0)
        )
        listener.waitForTransform(
            "/wrist_link", tool_frame, rospy.Time(), rospy.Duration(2.0)
        )
        (trans, _) = listener.lookupTransform(
            base_frame, "/arm_base_link", rospy.Time(0)
        )
        self.base_offset = trans

        (trans, _) = listener.lookupTransform(
            "/arm_base_link", "/elbow_link", rospy.Time(0)
        )
        self.l1 = trans[2]

        (trans, _) = listener.lookupTransform(
            "/elbow_link", "/forearm_link", rospy.Time(0)
        )
        self.l2 = math.sqrt(trans[0] ** 2 + trans[2] ** 2)
        self.angle2_bias = math.atan2(trans[0], trans[2])

        (trans, _) = listener.lookupTransform(
            "/forearm_link", "/wrist_link", rospy.Time(0)
        )
        self.l3 = trans[0]

        (trans, _) = listener.lookupTransform("/wrist_link", tool_frame, rospy.Time(0))
        self.l4 = trans[0]

        self.joint_limits = ([-pi, pi], [-pi, pi], [-pi, pi], [-pi, pi])

    def circle_intersection(self, circle1, circle2):
        """
        @summary: calculates intersection points of two circles
        @param circle1: tuple(x,y,radius)
        @param circle2: tuple(x,y,radius)
        @result: tuple of intersection points (which are (x,y) tuple)
        """
        # return self.circle_intersection_sympy(circle1,circle2)
        x1, y1, r1 = circle1
        x2, y2, r2 = circle2
        # http://stackoverflow.com/a/3349134/798588
        dx, dy = x2 - x1, y2 - y1
        d = math.sqrt(dx * dx + dy * dy)
        if d > r1 + r2:
            return None  # no solutions, the circles are separate
        if d < abs(r1 - r2):
            return None  # no solutions because one circle is contained within the other
        if d == 0 and r1 == r2:
            return None  # circles are coincident and there are an infinite number of solutions

        a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
        h = math.sqrt(r1 * r1 - a * a)
        xm = x1 + a * dx / d
        ym = y1 + a * dy / d
        xs1 = xm + h * dy / d
        xs2 = xm - h * dy / d
        ys1 = ym - h * dx / d
        ys2 = ym + h * dx / d
        return (xs1, ys1), (xs2, ys2)

    def ik(self, position, alpha=0.0, cur_arm_config=4 * [0]):
        """
        # x,y,z are end effector pose with respect to the base link
        :param position: cartesian param in m [x,y,z]
        :param alpha: float (in radians)alpha is the last joint angle wrt ground that you want.. upward +ve
        :param cur_arm_config: list config of arm, this will be helpful for selecting one out of multiple solutions
        :return: list if found the solution otherwise None
        """
        trans = copy.deepcopy(position)
        assert len(trans) == 3
        for i in range(3):
            trans[i] = trans[i] - self.base_offset[i]
        (x, y, z) = tuple(trans)
        # for holding final solution
        theta = [0.0, 0.0, 0.0, 0.0]

        # 1) Convert it into cylindrical system and find base angle
        theta[0] = math.atan2(y, x)
        if not self.joint_limits[0][0] < theta[0] < self.joint_limits[0][1]:
            return None

        # 2) As X,Y,Z & alpha for end-effector is given, therefore J4 location is fixed. now we neet to find out J3 & J2
        # location and use the locations to get the joint angles using circle circle intersection
        X = math.sqrt(x ** 2 + y ** 2)
        Y = z
        p3 = (X - self.l4 * math.cos(alpha), Y - self.l4 * math.sin(alpha))

        circle1 = (0, self.l1, self.l2)
        circle2 = (p3[0], p3[1], self.l3)
        inter = self.circle_intersection(circle1, circle2)

        # if found the solution
        if inter:
            inter1 = inter[0]
            inter2 = inter[1]
        else:
            return None

        # if there is intersection then 2 possible solution will be possible
        angle1 = (
            math.atan2(inter1[0], inter1[1] - self.l1) - self.angle2_bias,
            math.atan2(inter2[0], inter2[1] - self.l1) - self.angle2_bias,
        )

        angle2 = (
            -math.atan2(p3[1] - inter1[1], p3[0] - inter1[0]) - angle1[0],
            -math.atan2(p3[1] - inter2[1], p3[0] - inter2[0]) - angle1[1],
        )

        angle3 = (-alpha - angle1[0] - angle2[0], -alpha - angle1[1] - angle2[1])

        # filter out one angle
        # 1) if both found solutions are in range select the one which is closest to current configuration of robot
        if (
            all(self.joint_limits[1][0] < i < self.joint_limits[1][1] for i in angle1)
            and all(
                self.joint_limits[2][0] < i < self.joint_limits[2][1] for i in angle2
            )
            and all(
                self.joint_limits[3][0] < i < self.joint_limits[3][1] for i in angle3
            )
        ):
            # take the one which is closer to the current arm configuration
            diff_0 = (
                abs(angle1[0] - cur_arm_config[1])
                + abs(angle2[0] - cur_arm_config[2])
                + abs(angle3[0] - cur_arm_config[3])
            )
            diff_1 = (
                abs(angle1[1] - cur_arm_config[1])
                + abs(angle2[1] - cur_arm_config[2])
                + abs(angle3[1] - cur_arm_config[3])
            )
            if diff_0 < diff_1:
                theta[1] = angle1[0]
                theta[2] = angle2[0]
                theta[3] = angle3[0]
                return theta
            else:
                theta[1] = angle1[1]
                theta[2] = angle2[1]
                theta[3] = angle3[1]
                return theta

        # 2) which ever satisfy constraints
        elif (
            self.joint_limits[1][0] < angle1[0] < self.joint_limits[1][1]
            and self.joint_limits[2][0] < angle2[0] < self.joint_limits[2][1]
            and self.joint_limits[3][0] < angle3[0] < self.joint_limits[3][1]
        ):
            theta[1] = angle1[0]
            theta[2] = angle2[0]
            theta[3] = angle3[0]
            return theta

        elif (
            self.joint_limits[1][0] < angle1[1] < self.joint_limits[1][1]
            and self.joint_limits[2][0] < angle2[1] < self.joint_limits[2][1]
            and self.joint_limits[3][0] < angle3[1] < self.joint_limits[3][1]
        ):
            theta[1] = angle1[1]
            theta[2] = angle2[1]
            theta[3] = angle3[1]
            return theta

        else:
            return None

    def ik_quat(self, trans, quat, current_config=5 * [0]):
        """

        :param trans: cartesian param in m [x,y,z]
        :param quat: quaternion [x,y,z,w] 
        :param current_config: [list of len 5] current config of robot
        :return: [joint angles in radians]
        """
        assert len(quat) == 4

        nquat = [quat[3], quat[0], quat[1], quat[2]]

        # yaw, pitch, roll = euler_from_quaternion(quat, axes='rzyx')
        test = nquat[1] * nquat[3] - nquat[0] * nquat[2]
        # check singularities
        # https://www.sedris.org/wg8home/Documents/WG80485.pdf
        if test < -0.49:
            pitch = pi / 2.0
            rollminusyaw = math.atan2(
                nquat[1] * nquat[2] - nquat[0] * nquat[3],
                nquat[1] * nquat[3] + nquat[0] * nquat[2],
            )
            yaw = math.atan2(
                trans[1] - self.base_offset[1], trans[0] - self.base_offset[0]
            )
            roll = rollminusyaw + yaw
        elif test > 0.49:
            pitch = -pi / 2.0
            rollplusyaw = math.atan2(
                nquat[1] * nquat[2] - nquat[0] * nquat[3],
                nquat[1] * nquat[3] + nquat[0] * nquat[2],
            )
            yaw = math.atan2(
                trans[1] - self.base_offset[1], trans[0] - self.base_offset[0]
            )
            roll = rollplusyaw - yaw
        else:
            pitch = math.asin(-2 * (nquat[1] * nquat[3] - nquat[0] * nquat[2]))
            roll = math.atan2(
                nquat[3] * nquat[2] + nquat[0] * nquat[1],
                0.5 - (nquat[1] ** 2 + nquat[2] ** 2),
            )
            yaw = math.atan2(
                nquat[1] * nquat[2] + nquat[0] * nquat[3],
                0.5 - (nquat[2] ** 2 + nquat[3] ** 2),
            )

        angles = self.ik(trans, -pitch, current_config)
        if angles:
            angles.append(-roll)
        return angles

    def get_ik(self, qinit, x, y, z, rx, ry, rz, rw):
        """
        Do the IK call.

        :param list of float qinit: Initial status of the joints as seed.
        :param float x: X coordinates in base_frame.
        :param float y: Y coordinates in base_frame.
        :param float z: Z coordinates in base_frame.
        :param float rx: X quaternion coordinate.
        :param float ry: Y quaternion coordinate.
        :param float rz: Z quaternion coordinate.
        :param float rw: W quaternion coordinate.

        :return: joint values or None if no solution found.
        :rtype: [joint angles in radians]
        """
        trans = [x, y, z]
        quat = [rx, ry, rz, rw]
        solution = self.ik_quat(trans, quat, qinit)
        return solution


if __name__ == "__main__":
    ik = AnalyticInverseKinematics()
