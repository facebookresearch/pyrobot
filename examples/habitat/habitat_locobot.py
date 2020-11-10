from pyrobot import Robot
import os

import numpy as np

import habitat_sim
from habitat_sim.utils import common as utils

from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

from pyrobot.utils.util import try_cv2_import
from cv_bridge import CvBridge

import rospy
import time

cv2 = try_cv2_import()

class HabitatLocobot:
    def __init__(self):
        rospy.init_node("habitat_locobot")

        relative_path = "scenes/skokloster-castle.glb"
        common_config = dict(scene_path=relative_path)
        self.bot = Robot("habitat", common_config=common_config)
        self.base = self.bot.base
        self.sim = self.base.sim
        self.agent = self.sim.agents[0]

        self.ctrl_rate = 100
        self.realtime_scale_factor = 10
        self.interval = rospy.Duration.from_sec(1/self.ctrl_rate)
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        self.sim_time = rospy.Time()
        clock = Clock()
        self.clock_pub.publish(clock)

        self.img_rate = 25
        self.cv_bridge = CvBridge()
        self.rgb_img = None
        self.rgb_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)

        self.lin_speed = 0.
        self.ang_speed = 0.

        rospy.Subscriber(
            "/test_base_vel",
            Twist,
            self._command_callback,
        )

        self.vel_control = habitat_sim.physics.VelocityControl()
        self.vel_control.controlling_lin_vel = True
        self.vel_control.lin_vel_is_local = True
        self.vel_control.controlling_ang_vel = True
        self.vel_control.ang_vel_is_local = True

    def visualize(self):
        rgb_img = self.bot.camera.get_rgb()
        cv2.imshow("Color", rgb_img[..., 0:3][..., ::-1])
        base_state = self.bot.base.get_state()
        print("Base State:")
        print(base_state)
        cv2.waitKey(4000)

    def _command_callback(self, msg):
        self.lin_speed = msg.linear.x
        self.ang_speed = msg.angular.z

    def base_step(self):
        self.vel_control.linear_velocity = np.array([0,0,-self.lin_speed])
        self.vel_control.angular_velocity = np.array([0,self.ang_speed,0])

        state = self.agent.state
        previous_rigid_state = habitat_sim.RigidState(utils.quat_to_magnum(state.rotation), state.position)
        target_rigid_state = self.vel_control.integrate_transform(1/self.ctrl_rate, previous_rigid_state)
        end_pos = self.sim.step_filter(previous_rigid_state.translation, target_rigid_state.translation)
        state.position = end_pos
        state.rotation = utils.quat_from_magnum(target_rigid_state.rotation)
        self.agent.set_state(state)
        
        dist_moved_before_filter = (
            target_rigid_state.translation - previous_rigid_state.translation
        ).dot()
        dist_moved_after_filter = (
            end_pos - previous_rigid_state.translation
        ).dot()
        EPS = 1e-5
        collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter
        self.sim.step_physics(1/self.ctrl_rate)

        self.sim_time = self.sim_time + self.interval
        clock = Clock()
        clock.clock = self.sim_time
        self.clock_pub.publish(clock)
        # self.visualize()
        base_state = self.base.get_state()
        print("Base State:")
        print(base_state)

    def img_step(self):
        self.rgb_img = self.bot.camera.get_rgb()
        try:
            rgb_img_msg = self.cv_bridge.cv2_to_imgmsg(self.rgb_img, encoding="bgr8")
            self.rgb_pub.publish(rgb_img_msg)
        except Exception as e:
            rospy.logerr(e)

    def spin(self):
        prev_time = time.time()
        while not rospy.is_shutdown():
            self.base_step()
            if time.time() - prev_time > 1/(self.img_rate * self.realtime_scale_factor):
                self.img_step()
            time.sleep(1/(self.ctrl_rate * self.realtime_scale_factor))

if __name__ == "__main__":
    server = HabitatLocobot()
    server.spin()