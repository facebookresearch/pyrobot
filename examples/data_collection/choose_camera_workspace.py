# to choose the workspace in image space interactively
import time
import numpy as np
import argparse
from pyrobot import Robot
import sys
from IPython import embed
from termcolor import colored


class CamWs(object):
    def __init__(self,args):
        self.ros_cv_path = args.ros_cv_path
        self.camera = Robot(args.cam_name)

    def _nothing(self, x):
        pass

    def choose_cam_ws(self):
        if args.ros_cv_path in sys.path:
            sys.path.remove(args.ros_cv_path)
            import cv2
        sys.path.append(args.ros_cv_path)
        print(colored('First adjust X & Y and then H & W', 'red'))
        print(colored('Press ESC button after you are done', 'red'))

        time.sleep(2)
        start = False
        rgb = self.camera.camera.get_rgb()
        H, W, _ = rgb.shape
        x, y, h, w = 0, 0, H, W
        while True:
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            rgb = self.camera.camera.get_rgb()
            rgb_crop = np.zeros_like(rgb)
            rgb_crop[x:x + h, y:y + w] = rgb[x:x + h, y:y + w][:, :, ::-1]
            cv2.imshow('test', rgb_crop)
            if not start:
                cv2.createTrackbar('X', 'test', x, H, self._nothing)
                cv2.createTrackbar('Y', 'test', y, W, self._nothing)
                cv2.createTrackbar('H', 'test', h, H, self._nothing)
                cv2.createTrackbar('W', 'test', w, W, self._nothing)
                start = True

            x = cv2.getTrackbarPos('X', 'test')
            y = cv2.getTrackbarPos('Y', 'test')
            h = cv2.getTrackbarPos('H', 'test')
            w = cv2.getTrackbarPos('W', 'test')

        cv2.destroyAllWindows()
        print("x_left_top = {}, y_left_top = {}, x_right_bottom = {}, y_right_bottom = {}".format(x, y, x + h, y + w))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Args for choosing the workspace in camera space")
    parser.add_argument('--ros-cv-path', help='path to the ros opencv package',type=str,
                        default='/opt/ros/kinetic/lib/python2.7/dist-packages')
    parser.add_argument('--cam-name', help='name of the camera', type=str,default='kinect2')

    args = parser.parse_args()
    cam_ws = CamWs(args)
    cam_ws.choose_cam_ws()

