import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image

from cv_bridge import CvBridge

from pyrobot.utils.util import try_cv2_import
cv2 = try_cv2_import()

class HabitatInterface(object):
    """
    This is a parent class on which the robot
    specific Base classes would be built.
    """
    rospy.init_node("habitat_interface")

    def __init__(self):
        """
        The consturctor for Base class.

        :param configs: configurations for base
        :type configs: YACS CfgNode
        """
        self.ctrl_pub = rospy.Publisher(
            "/test_base_vel", Twist, queue_size=1
        )

        self.rgb_img = None
        self.cv_bridge = CvBridge()
        rospy.Subscriber(
            '/camera/color/image_raw',
            Image,
            self._rgb_callback,
        )

    def _rgb_callback(self, rgb):
        self.rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")

    def stop(self):
        """
        Stop the base
        """
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.ctrl_pub.publish(msg)

    def visualize(self):
        cv2.imshow("Color", self.rgb_img[..., 0:3][..., ::-1])
        cv2.waitKey(4000)

    def set_vel(self, fwd_speed, turn_speed, exe_time=1):
        """
        Set the moving velocity of the base

        :param fwd_speed: forward speed
        :param turn_speed: turning speed
        :param exe_time: execution time
        """

        msg = Twist()
        msg.linear.x = fwd_speed
        msg.angular.z = turn_speed

        start_time = rospy.get_time()
        print(rospy.get_time())
        self.ctrl_pub.publish(msg)
        while rospy.get_time() - start_time < exe_time:
            print(rospy.get_time())
            self.ctrl_pub.publish(msg)
            rospy.sleep(1.0 / 10)
        self.stop()

if __name__ == "__main__":
    interface = HabitatInterface()
    interface.set_vel(1, 0)
    interface.visualize()