# agent for collecting pushing and grasping data
from pyrobot import Robot
from termcolor import colored
import time
import copy
import numpy as np
from pyrobot.utils.util import MoveitObjectHandler, convert_frames, euler_to_quat
from math import pi
import os
from kbhit import KBHit
from IPython import embed
import threading
import random
import argparse
import yaml
import pickle

# TODO: Need to add gripper state function for gripper in sawyer --> [Done]
# TODO: Need to add image visualizer --> [Done]
# TODO: add the table adding part --> [Done]
# TODO: Need to add velocity control while motion planning


class Agent(object):

    def __init__(self, args):
        self.arm = Robot(args.arm_name)
        self.cam = Robot(args.cam_name)
        self.z_min = args.z_min
        self.arm_base_frame = args.arm_base_frame
        self.camera_img_frame = args.cam_img_frame
        self.on_top_height = args.on_top_height
        self.num_angle = args.num_angle          # if set to <= 0, will be considered continues
        self.depth_bin_thr = args.depth_bin_thr
        self.erosion_kernel_size = args.erosion_kernel_size
        self.erosion_iter = args.erosion_iter
        self.empty_thr = args.empty_thr

        with open(args.img_region_yaml, 'r') as f:
            crop_data = yaml.load(f, Loader=yaml.FullLoader)
        self.crop_region = {'top_left': crop_data['top_left'], 'bottom_right': crop_data['bottom_right']}

        with open(args.table_yaml, 'r') as f:
            self.table_param = yaml.load(f, Loader=yaml.FullLoader)

        with open(args.push_pix_range_yaml, 'r') as f:
            push_pix_range = yaml.load(f, Loader=yaml.FullLoader)

        self._push_pix_range = push_pix_range['range']
        self._img_vis_scale = 2.0
        self.object_in_hand_state = 2
        self._action = None
        self._action_param = None

        self._img_thread = threading.Thread(target=self._show_img, args=())
        self._img_thread.daemon = True  # Daemonize thread

    def grasp(self, action_param):
        """
        
        :param action_param:
        :return:
        """

        img_pt = action_param['loc']
        grasp_ang = action_param['ang']
        result = {'img': copy.deepcopy(self._crop_img(self.cam.camera.get_rgb())),
                  'loc': self._org_2_crop(img_pt), 'ang': grasp_ang}

        # check if gripper is holding any object ###################
        self.arm.gripper.close()
        if self.arm.gripper.get_gripper_state() == self.object_in_hand_state:     # TODO: Check if its right
            inp = input(colored("Looks like there is object in hand , remove objects and  hit enter continue\n",
                                "red"))
            time.sleep(5)
        self.arm.gripper.open()

        # convert 2D target in image to 3D target in robot
        # TODO: Note the indx should be proper
        img_pt_3D, _ = list(self.cam.camera.pix_to_3dpt(img_pt[0], img_pt[1], reduce='mean'))
        robot_pt_3D = convert_frames(np.squeeze(img_pt_3D), self.camera_img_frame, self.arm_base_frame)

        # orientation #########################
        # assumption is for 0 angle the gripper long edge will be parallel to long edge of table --> CHECKED
        grasp_orientation = list(euler_to_quat([grasp_ang, 0.0, pi]))

        # go on top of the point ##############
        print("going on top of the object")
        self.arm.arm.set_ee_pose(plan=True, position=np.array([robot_pt_3D[0], robot_pt_3D[1], self.on_top_height]),
                                 orientation=np.array(grasp_orientation))

        # go down in cartesian space ##########
        print("going to grab object")
        self.arm.arm.move_ee_xyz(np.array([0.0, 0.0, self.z_min - self.on_top_height]), plan=True)

        # close gripper #######################
        print("closing the gripper")
        self.arm.gripper.close()

        # come up #############################
        print("coming up")
        self.arm.arm.move_ee_xyz(np.array([0.0, 0.0, self.on_top_height - self.z_min]), plan=True)

        # check gripper condition #############
        grasped = self.arm.gripper.get_gripper_state() == self.object_in_hand_state

        # if it is grasped , place it at some random location ###########
        if grasped:
            print("going to random location to drop the object")
            # get random location on the same plane
            self.arm.arm.move_ee_xyz(np.array([0.1*(np.random.random()-0.5), 0.1*(np.random.random()-0.5), 0.0]),
                                     plan=True)
            self.arm.gripper.open()
            time.sleep(2)

        # come back to mean position ##########
        print("going to mean position")
        self.arm.arm.move_to_neutral()

        result.update({'grasped': grasped})
        return result

    def push(self, action_param):
        """

        :param action_param:
        :return:
        """

        result = {'start_img': copy.deepcopy(self._crop_img(self.cam.camera.get_rgb())),
                  'start_pt': self._org_2_crop(action_param['start_pt']),
                  'end_pt': self._org_2_crop(action_param['end_pt'])}

        # check if gripper is holding any object ###################
        self.arm.gripper.close()
        if self.arm.gripper.get_gripper_state() == self.object_in_hand_state:     # TODO: Check if its right
            inp = input(colored("Looks like there is object in hand , remove objects and  hit enter continue\n",
                                "red"))
            time.sleep(5)

        # convert 2D target in image to 3D target in robot
        # TODO: Note the indx should be proper
        start_pt = action_param['start_pt']
        end_pt = action_param['end_pt']
        start_img_pt_3D, _ = list(self.cam.camera.pix_to_3dpt(start_pt[0], start_pt[1], reduce='mean'))
        start_robot_pt_3D = convert_frames(np.squeeze(start_img_pt_3D), self.camera_img_frame, self.arm_base_frame)

        end_img_pt_3D, _ = list(self.cam.camera.pix_to_3dpt(end_pt[0], end_pt[1], reduce='mean'))
        end_robot_pt_3D = convert_frames(np.squeeze(end_img_pt_3D), self.camera_img_frame, self.arm_base_frame)

        push_ang = np.arctan2(end_robot_pt_3D[1] - start_robot_pt_3D[1], end_robot_pt_3D[0] - start_robot_pt_3D[0])
        push_orientation = list(euler_to_quat([push_ang, 0.0, pi]))
        # go on top of the point ##############
        print("going on top of the object")
        self.arm.arm.set_ee_pose(plan=True, position=np.array([start_robot_pt_3D[0], start_robot_pt_3D[1], self.on_top_height]),
                                 orientation=np.array(push_orientation))

        # go down in cartesian space ##########
        print("going to start_loc")
        self.arm.arm.move_ee_xyz(np.array([0.0, 0.0, self.z_min - self.on_top_height]), plan=True)
        time.sleep(2)

        # going to end push point #######################
        self.arm.arm.move_ee_xyz(np.array([end_robot_pt_3D[0] - start_robot_pt_3D[0],
                                           end_robot_pt_3D[1] - start_robot_pt_3D[1],
                                           0.0]), plan=True)
        time.sleep(2)

        # come up #############################
        print("coming up")
        self.arm.arm.move_ee_xyz(np.array([0.0, 0.0, self.on_top_height - self.z_min]), plan=True)
        time.sleep(2)

        # come back to mean position ##########
        print("going to mean position")
        self.arm.arm.move_to_neutral()

        result.update({'end_img': copy.deepcopy(self._crop_img(self.cam.camera.get_rgb()))})
        return result

    def collect_data(self, action='grasp', mode='object_centric', store_path='./temp', num_data=1000,
                     meta_file='metadata.txt'):
        """

        :param action:
        :param mode: string ['random', 'object_centric']
        :param store_path:
        :param num_data:
        :param meta_file:
        :return:
        """
        # check if path exist ###################
        if not os.path.isdir(store_path):
            os.makedirs(store_path)

        # check if metadata file exist
        if not os.path.isfile(os.path.join(store_path, meta_file)):
            with open(os.path.join(store_path, meta_file), "w") as f:
                f.write('0')

        with open(os.path.join(store_path, meta_file), "r") as f:
            count = int(f.read())

        # Remove all the objects from the table ######
        inp = input(colored("Remove all objects on the table and press s key and  enter", "red"))
        while inp != 's':
            inp = input(colored("Looks like you have pressed {} key, press s key and hit enter to continue".format(
                    inp), "red"))
        time.sleep(2)

        # move arm to mean position ###################
        self.arm.arm.move_to_neutral()
        self.arm.gripper.close()
        time.sleep(2)

        # get the mean depth of the img region which is used for data collection #######
        num_capture = 10
        for i in range(num_capture):
            if i == 0:
                self.mean_depth = self._crop_img(self.cam.camera.get_depth().astype(np.float32))
            else:
                self.mean_depth += self._crop_img(self.cam.camera.get_depth().astype(np.float32))

        self.mean_depth /= num_capture

        # Place objects on the table ######
        inp = input(colored("Place objects on table and press s key and  enter", "green"))
        while inp != 's':
            inp = input(colored("Looks like you have pressed {} key, press s key and hit enter to continue".format(
                    inp), "green"))
        time.sleep(2)

        # start visualization ############################
        self._img_thread.start()

        # add table in moveit environment ################
        obstacle_handler = MoveitObjectHandler()
        obstacle_handler.add_table(self.table_param['pose'], tuple(self.table_param['size']))

        self.arm.arm.move_to_neutral()
        kbhit = KBHit()
        while count < num_data:
            # give the option to pause process in between to rearrange objects ######
            if kbhit.kbhit():
                c = kbhit.getch()
                if c == 'p':
                    inp = input(
                        colored("Looks like you have paused the process, place objects and press s key and hit enter "
                                "continue\n", "red"))
                    while inp != 's':
                        inp = input(colored(
                                "Looks like you have pressed {} key, press s key and hit enter to continue".format(
                                    inp),"red"))

            # check if there are objects on table #######
            if self._table_empty():
                input(colored("Looks like there is no object on table, place some objects and press enter","red"))

            self._action = action
            # sample location based on mode
            if mode in ['random', 'object_centric']:
                self._action_param = self._get_action_param(action=action, mode=mode)
            else:
                raise ValueError("mode can either be 'random' or 'object centric'")

            # perform action based on the action category ######
            if action == 'grasp':
                result = self.grasp(self._action_param)
            elif action == 'push':
                result = self.push(self._action_param)
            else:
                raise ValueError("action can either be 'grasp' or 'push'")

            print("\n############")
            print("collected transition num = " + colored('{:04d}'.format(count), 'green'))
            print("############")
            count += 1

            # store data ##########################
            with open(os.path.join(store_path, "{}.pk".format(count)), "wb") as f:
                pickle.dump(result, f, protocol=2)

            # store count in file
            with open(os.path.join(store_path, meta_file), "w") as f:
                f.write(str(count))

            time.sleep(2)

    def _org_2_crop(self, pt):
        """
        helpful for converting image point from original image to cropped region
        :param pt: [X,Y]
        :return: [cropped_X, cropped_Y]
        """
        return [pt[0]-self.crop_region['top_left'][0], pt[1]-self.crop_region['top_left'][1]]

    def _crop_2_org(self, pt):
        """
        helpful for converting image point from original image to cropped region
        :param pt: [cropped_X, cropped_Y]
        :return: [X, Y]
        """
        return [pt[0] + self.crop_region['top_left'][0], pt[1] + self.crop_region['top_left'][1]]

    def _crop_img(self, img):
        """

        :param img: np.array([width, height, channel])
        :return: cropped img
        """
        return img[self.crop_region['top_left'][0]:self.crop_region['bottom_right'][0],
               self.crop_region['top_left'][1]:self.crop_region['bottom_right'][1]]

    def _table_empty(self):
        """
        :return: True if table is empty .. otherwise False
        """
        diff_erosion = self._get_depth_bin()

        if diff_erosion.mean() > self.empty_thr:
            return False
        else:
            return True

    def _get_depth_bin(self):
        """
        :return: returns the binary uint8{0,255} image out
        """
        diff = self.mean_depth.astype(np.float32) - self._crop_img(self.cam.camera.get_depth().astype(np.float32))
        diff[diff < self.depth_bin_thr] = 0
        diff[diff != 0] = 255
        diff = diff.astype(np.uint8)

        # erode the image
        kernel = np.ones((self.erosion_kernel_size, self.erosion_kernel_size), np.uint8)
        diff_erosion = cv2.erode(diff, kernel, iterations=self.erosion_iter)

        # dilate
        diff_erosion = cv2.dilate(diff_erosion, kernel, iterations=self.erosion_iter)
        return copy.deepcopy(diff_erosion)

    def _vis_grasp(self, img, loc, grasp_angle):
        """

        :param img:
        :param loc:
        :param grasp_angle:
        :return:
        """
        # input cv_image -->[h,w,c]
        vis_img = img.copy()
        gsize = 75
        grasp_l = gsize/2.5
        grasp_w = gsize/5.0
        grasp_angle *= -1.0
        points = np.array([[-grasp_l, -grasp_w],
                           [grasp_l, -grasp_w],
                           [grasp_l, grasp_w],
                           [-grasp_l, grasp_w]])
        R = np.array([[np.cos(grasp_angle), -np.sin(grasp_angle)],
                      [np.sin(grasp_angle), np.cos(grasp_angle)]])
        rot_points = np.dot(R, points.transpose()).transpose()
        im_points = rot_points + np.array([loc[1], loc[0]])          # indices swaped to transfer numpy pt to cv pt
        cv2.line(vis_img, tuple(im_points[0].astype(int)), tuple(im_points[1].astype(int)), color=(0, 255, 0),
                 thickness=3)
        cv2.line(vis_img, tuple(im_points[1].astype(int)), tuple(im_points[2].astype(int)), color=(0, 0, 255),
                 thickness=3)
        cv2.line(vis_img, tuple(im_points[2].astype(int)), tuple(im_points[3].astype(int)), color=(0, 255, 0),
                 thickness=3)
        cv2.line(vis_img, tuple(im_points[3].astype(int)), tuple(im_points[0].astype(int)), color=(0, 0, 255),
                 thickness=3)
        return vis_img

    def _vis_push(self, img, start_pt, end_pt):
        """

        :param img:
        :param start_pt:
        :param end_pt:
        :return:
        """
        img = cv2.arrowedLine(img, (start_pt[1], start_pt[0]), (end_pt[1], end_pt[0]), color=(0, 0, 255),
                        thickness=3, tipLength=0.2).get()
        return img

    def _get_action_param(self, action, mode):
        """

        :param action:
        :param mode:
        :return:
        """
        if self.num_angle <= 0:
            ang = np.random.random() * 2 * pi
        else:
            ang = np.random.randint(0, self.num_angle) * 2 * pi / float(self.num_angle)
        if mode == 'random':
            pt = [np.random.randint(self.crop_region['top_left'][0], self.crop_region['bottom_right'][0]),
                  np.random.randint(self.crop_region['top_left'][1], self.crop_region['bottom_right'][1])]
            if action == 'grasp':
                action_param = {'loc': pt, 'ang': ang}
            elif action == 'push':
                # sample another point in at some pixel distance from
                if np.random.random() < 0.5:
                    pt_2 = list(np.array(pt) - np.random.randint(self._push_pix_range[0], self._push_pix_range[1], 2))
                else:
                    pt_2 = list(np.array(pt) - np.random.randint(-self._push_pix_range[1], self._push_pix_range[0], 2))
                action_param = {'start_pt': pt, 'end_pt': [int(pt_2[0]), int(pt_2[1])]}
        elif mode == 'object_centric':
            bin_img = self._get_depth_bin()
            object_tuple = np.where(bin_img == 255)
            indx = np.random.randint(0, len(object_tuple))

            if action == 'grasp':
                action_param = {'loc': self._crop_2_org([int(object_tuple[0][indx]), int(object_tuple[1][indx])]),
                                'ang': ang}

            elif action == 'push':
                min_search_range = [15, 15]
                max_search_range = [20, 20]
                x_max, y_max = bin_img.shape
                while True:
                    X = np.clip(int(object_tuple[0][indx]) + random.randrange(-1, 2, step=2) *
                                np.random.randint(min_search_range[0], max_search_range[0]), 0, x_max)
                    Y = np.clip(int(object_tuple[1][indx]) + random.randrange(-1, 2, step=2) *
                                np.random.randint(min_search_range[1], max_search_range[1]), 0, y_max)
                    if bin_img[X][Y] == 0:
                        not_object_indx = np.array([X, Y])
                        break

                start = not_object_indx
                end = [np.clip(2 * object_tuple[0][indx] - start[0], 0, x_max),
                       np.clip(2 * object_tuple[1][indx] - start[1], 0, y_max)]
                action_param = {'start_pt': self._crop_2_org([int(start[0]), int(start[1])]),
                                'end_pt':self._crop_2_org([int(end[0]), int(end[1])])}
        return action_param

    def _show_img(self):
        """

        :return:
        """
        while True:
            img = self.cam.camera.get_rgb()[:, :, ::-1]
            if self._action is not None and self._action_param is not None:
                if self._action == 'grasp':
                    img = self._vis_grasp(img, self._action_param['loc'], self._action_param['ang'])
                elif self._action == 'push':
                    img = self._vis_push(img, self._action_param['start_pt'], self._action_param['end_pt'])
            cv2.imshow('img', cv2.resize(self._crop_img(img), (0, 0), fx=self._img_vis_scale,
                                         fy=self._img_vis_scale))
            cv2.waitKey(33)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Args for collecting grasping or pushing data using robot")
    parser.add_argument('--arm-name', help='name of the arm', type=str, default='sawyer')
    parser.add_argument('--cam-name', help='name of the camera', type=str, default='kinect2')
    parser.add_argument('--arm-base-frame', help='name of the arm base frame', type=str, default='/base')
    parser.add_argument('--cam-img-frame', help='name of the image frame', type=str,
                        default='/kinect2_rgb_optical_frame')
    parser.add_argument('--z-min', help='z position of hand when gripper is in vertically down position and finger tip '
                                        'touches table', type=float, default=-0.15)
    parser.add_argument('--on-top-height', help='z position of hand before/after pushing/gasping action has been '
                                                'executed', type=float, default=0.1)
    parser.add_argument('--num-angle', help='the number of grasping angles (if <0, angles will be continues)',
                        type=int, default=18)
    parser.add_argument('--depth-bin-thr', help='threshold used to convert depth img (with table depth subtraction) '
                                                'into binary image for object detection on table (should be +ve)',
                        type=int, default=10)
    parser.add_argument('--erosion-kernel-size', help='erosion kernel size that will be applied to mean subtracted '
                                                      'depth image', type=int, default=2)
    parser.add_argument('--erosion-iter', help='number of times erosion operation should applied on depth subtracted '
                                               'image', type=int, default=2)
    parser.add_argument('--empty-thr', help='mean of bin depth image to be considered whether table is empty or not',
                        type=int, default=1.0)
    parser.add_argument('--action', help='for which data needs to be collected [grasp, push]', type=str,
                        default='grasp')
    parser.add_argument('--store-path', help='path to store data', type=str, default='./temp')
    parser.add_argument('--num-data-point', help='num of data points to collect', type=int, default=100)
    parser.add_argument('--ros-cv-path', help='path to the ros opencv package', type=str,
                        default='/opt/ros/kinetic/lib/python2.7/dist-packages')
    parser.add_argument('--img-region-yaml', help='region to crop from img', type=str, default='img_region.yaml')
    parser.add_argument('--table-yaml', help='pose and size of the table to put in environment for planning', type=str,
                        default='table.yaml')
    parser.add_argument('--push-pix-range-yaml', help='pushing range in pix', type=str, default='push_pix_range.yaml')
    parser.add_argument('--mode', help='mode for data collection [random, object_centric]', type=str,
                        default='object_centric')
    args = parser.parse_args()

    import sys
    if args.ros_cv_path in sys.path:
        sys.path.remove(args.ros_cv_path)
        import cv2
    sys.path.append(args.ros_cv_path)

    agent = Agent(args)
    agent.collect_data(action=args.action, mode=args.mode, store_path=args.store_path, num_data=args.num_data_point)