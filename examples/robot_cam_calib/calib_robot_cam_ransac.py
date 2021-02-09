# for comeplte calibratio, in this we will consider base(B), camera(C), gripper(G) and AR Tag(A) frames
# trasform from B<-->G and C<-->A is known
# need to figure out transform between G<-->A and B<-->C
# P_X_Y --> represent origin of Y frame in X frame of reference

import torch
from torch import optim
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import matplotlib
from util import quat2mat, quat2rot, compute_loss
import argparse
import os
import pickle
import random

class GetCameraExtrensics(object):

    def __init__(self, args):
        """

        :param args:
        """
        self.data_dir = args.data_dir
        self.rot_loss_w =  args.rot_loss_w
        self.vis = args.vis
        self.num_iter = args.num_iter
        self.base_frame = args.base_frame
        self.camera_frame = args.camera_frame
        self.num_train_points = args.num_data_points
        self.num_ransac_iter = args.num_ransac_iter
        self.inlier_thr = args.inlier_thr

    def calibrate(self):
        """

        :return:
        """
        # generate data
        self.load_data(self.data_dir)
        # optimize the parmas
        self.optimize_ransac(self.num_iter, self.rot_loss_w, self.num_ransac_iter, self.inlier_thr, self.vis)

    # data set generator
    def load_data(self, data_dir):
        """

        :param data_dir:
        :return:
        """
        # load hand data
        with open(os.path.join(data_dir, 'arm_2.p'), 'rb') as f:
            try:
                arm_data = pickle.load(f)['data']
            except:
                arm_data = pickle.load(f, encoding='latin1')['data']

        # load marker data
        with open(os.path.join(data_dir, 'marker_2.p'), 'rb') as f:
            try:
                marker_data = pickle.load(f)['data']
            except:
                marker_data = pickle.load(f, encoding='latin1')['data']
        self.num_points = min(len(arm_data),len(marker_data))
        self.trans_B_G = torch.from_numpy(np.array([arm_data[i]['position'] for i in range(self.num_points)])
                                          .reshape(-1, 3))
        self.rot_B_G = torch.from_numpy(np.array([arm_data[i]['orientation'] for i in range(self.num_points)]))
        self.trans_C_A = torch.from_numpy(np.array([marker_data[i]['position'] for i in range(self.num_points)]).
                                          reshape(-1, 3))
        quat_C_A = torch.from_numpy(np.array([marker_data[i]['orientation'] for i in range(self.num_points)]))
        self.rot_C_A = quat2rot(quat_C_A, format='xyzw')
        self.num_points = self.trans_B_G.shape[0]

    # optimize the parameters
    def optimize_ransac(self, num_iter, rot_loss_w, num_ransac_iter, inlier_thr, vis):
        """

        :param num_iter:
        :param rot_loss_w:
        :param num_ransac_iter:
        :param inlier_thr:
        :param vis:
        :return:
        """
        max_inliers = None
        for n in range(num_ransac_iter):
            # sample random num_points from data to optimize paramters
            print("\n training with {} data points".format(self.num_train_points))
            train_indx = random.sample(range(self.num_points), self.num_train_points)
            train_trans_B_G = torch.stack([self.trans_B_G[i] for i in train_indx], dim=0)
            train_rot_B_G = torch.stack([self.rot_B_G[i] for i in train_indx], dim=0)
            train_trans_C_A = torch.stack([self.trans_C_A[i] for i in train_indx], dim=0)
            train_rot_C_A = torch.stack([self.rot_C_A[i] for i in train_indx], dim=0)

            test_trans_B_G = torch.stack([self.trans_B_G[i] for i in range(self.num_points) if i not in train_indx],
                                         dim=0)
            test_rot_B_G = torch.stack([self.rot_B_G[i] for i in range(self.num_points) if i not in train_indx], dim=0)
            test_trans_C_A = torch.stack([self.trans_C_A[i] for i in range(self.num_points) if i not in train_indx],
                                         dim=0)
            test_rot_C_A = torch.stack([self.rot_C_A[i] for i in range(self.num_points) if i not in train_indx], dim=0)


            # start with some random guess
            quat_B_C = torch.rand(1,3).double().requires_grad_(True)
            trans_B_C = torch.rand(1,3).double().requires_grad_(True)
            quat_G_A = torch.rand(1,3).double().requires_grad_(True)
            trans_G_A = torch.rand(1,3).double().requires_grad_(True)
            optimizer = optim.Adam([quat_B_C, trans_B_C, trans_G_A, quat_G_A], lr=0.1)
            criterion = torch.nn.MSELoss(reduction='none')
            best_train_loss, best_train_quat_B_C, best_train_trans_B_C, best_train_quat_G_A, best_train_trans_G_A = \
                None, None, None, None, None

            ###################
            # optimize on the train set the B<-->C & G<-->A
            for it in range(num_iter):
                _, train_loss = compute_loss(train_trans_B_G, train_rot_B_G, train_trans_C_A, train_rot_C_A, trans_G_A,
                                       quat_G_A, trans_B_C, quat_B_C, criterion, rot_loss_w)
                optimizer.zero_grad()
                train_loss.backward()
                optimizer.step()

                if best_train_loss is None or train_loss.item() < best_train_loss:
                    best_train_loss = train_loss.item()
                    best_train_quat_B_C = quat_B_C.detach().numpy()
                    best_train_trans_B_C = trans_B_C.detach().numpy()
                    best_train_quat_G_A = quat_G_A.detach().numpy()
                    best_train_trans_G_A = trans_G_A.detach().numpy()

                if it % 100 == 0:
                    print("train_loss = {:05f}".format(train_loss.item()))

            ###################
            # find inliers
            with torch.no_grad():
                test_loss, _ = compute_loss(test_trans_B_G, test_rot_B_G, test_trans_C_A, test_rot_C_A,
                                            torch.from_numpy(best_train_trans_G_A),
                                            torch.from_numpy(best_train_quat_G_A),
                                            torch.from_numpy(best_train_trans_B_C),
                                            torch.from_numpy(best_train_quat_B_C), criterion, rot_loss_w)

            # include all inliers in train set
            num_inliers = 0
            for indx, l in enumerate(test_loss):
                if l.item() < inlier_thr:
                    train_trans_B_G = torch.cat((train_trans_B_G, test_trans_B_G[indx].unsqueeze_(0)), dim=0)
                    train_rot_B_G = torch.cat((train_rot_B_G, test_rot_B_G[indx].unsqueeze_(0)), dim=0)
                    train_trans_C_A = torch.cat((train_trans_C_A, test_trans_C_A[indx].unsqueeze_(0)), dim=0)
                    train_rot_C_A = torch.cat((train_rot_C_A, test_rot_C_A[indx].unsqueeze_(0)), dim=0)
                    num_inliers += 1
            print("num_inliers = {}".format(num_inliers))

            # fine tune the params
            if num_inliers == 0:
                continue
            if max_inliers is None or num_inliers > max_inliers:
                max_inliers = num_inliers
                print("training with {} data points".format(train_trans_B_G.shape[0]))
                # train again
                best_loss, best_quat_B_C, best_trans_B_C, best_quat_G_A, best_trans_G_A = None, None, None, None, None
                for it in range(num_iter):
                    # optimize paramters
                    optimizer.zero_grad()
                    _, train_loss = compute_loss(train_trans_B_G, train_rot_B_G, train_trans_C_A, train_rot_C_A,
                                                 trans_G_A,
                                                 quat_G_A, trans_B_C, quat_B_C, criterion, rot_loss_w)
                    if best_loss is None or train_loss.item() < best_loss:
                        best_loss = train_loss.item()
                        best_quat_B_C = quat_B_C.detach().numpy()
                        best_trans_B_C = trans_B_C[0].detach().numpy()
                        best_trans_G_A = trans_G_A[0].detach().numpy()
                    train_loss.backward()
                    optimizer.step()
                    if it % 100 == 0:
                        print("train_loss = {:05f}".format(train_loss.item()))

        best_rot_B_C, best_quat_B_C = quat2mat(torch.from_numpy(best_quat_B_C))
        best_rot_B_C, best_quat_B_C = best_rot_B_C[0].detach().numpy(), best_quat_B_C[0].detach().numpy()

        print("\n for B<-->C ")
        cmd = "rosrun tf static_transform_publisher " + str(float(best_trans_B_C[0])) + ' ' + \
              str(float(best_trans_B_C[1])) + ' ' + str(float(best_trans_B_C[2])) + ' ' + str(best_quat_B_C[1]) + ' ' \
              + str(best_quat_B_C[2]) + ' ' + str(best_quat_B_C[3]) + ' ' + str(best_quat_B_C[0]) + ' ' + \
              self.base_frame + ' '+ self.camera_frame + ' 10'
        print("Run Command")
        print(cmd)

        # plot the points for visualization
        if vis:
            trans_B_G_A = self.trans_B_G.numpy().reshape(-1,3) + np.array([np.matmul(self.rot_B_G[i].numpy(),
                                                                                     best_trans_G_A.reshape(-1,3).T).T
                                                                           for i in range(self.num_points)]).reshape(-1,3)
            trans_B_C_A = np.matmul(best_rot_B_C,self.trans_C_A.numpy().reshape(-1,3).T).T + best_trans_B_C.reshape(-1,3)
            ax = plt.axes(projection='3d')
            ax.scatter3D(trans_B_G_A[:,0], trans_B_G_A[:,1], trans_B_G_A[:,2])
            ax.scatter3D(trans_B_C_A[:,0], trans_B_C_A[:,1], trans_B_C_A[:,2], color='red')
            scatter1_proxy = matplotlib.lines.Line2D([0],[0], linestyle="none", marker = 'o')
            scatter2_proxy = matplotlib.lines.Line2D([0],[0], linestyle="none", c='red', marker = 'o')
            ax.legend([scatter1_proxy, scatter2_proxy], ['Base to Ar from Gripper', 'Base to Ar from Camera'], numpoints = 1)
            plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process args for calibration")
    parser.add_argument('--rot_loss_w', help='weight on rotational loss for optimizing the camera extrensic parameters', type=float, default=0.0)
    parser.add_argument('--vis', action='store_true', default=False, help='for visualizing data pooints after calibration')
    parser.add_argument('--num_iter', help='number of iteration of optimization', type=int, default=1000)
    parser.add_argument('--data_dir', help='Directory to load data points', type=str, default="robot_ar_data")
    parser.add_argument('--base_frame', help='robot base frame name', type=str, default="/base")
    parser.add_argument('--camera_frame', help='camera frame name', type=str, default="/kinect2_rgb_optical_frame")
    parser.add_argument('--num_data_points', help='number of data points used to optimize the intial guess', type=int, default=5)
    parser.add_argument('--num_ransac_iter', help='number of data points used to optimize the intial guess', type=int,
                        default=20)
    parser.add_argument('--inlier_thr', help='the loss below which the point will be considered inlier', type=float,
                        default=0.01)

    args = parser.parse_args()
    get_camera_extrensics = GetCameraExtrensics(args)
    get_camera_extrensics.calibrate()