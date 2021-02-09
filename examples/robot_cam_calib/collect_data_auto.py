# Note: while collecting data using this code make sure that you have added obstacles in moveit envrionment according to your setup
import os
from IPython import embed
from pyrobot import Robot
import time
import pickle
import argparse
from util import ArMarker
from termcolor import colored
import numpy as np

class RobotArDataCollectorAuto(object):

    def __init__(self, args):
        self.robot = Robot('sawyer',use_base=False,use_camera=False)
        self.ar_marker = ArMarker(args.ar_id)
        self.delta = args.delta
        self.x_range = [float(i) for i in args.x_range]
        self.y_range = [float(i) for i in args.y_range]
        self.z_range = [float(i) for i in args.z_range]
        self.quat = [float(i) for i in args.quat]
        self.pause_time = args.pause_time

        # for storing the data
        self.data_dir = args.data_dir
        try:
            os.makedirs(self.data_dir)
        except:
            colored("{} directory already exists, may overwrite the data".format(self.data_dir),"yellow")

    # collect data
    def collect_data(self):
        arm_data_list = []
        marker_data_list = []
        count = 0
        for z in np.arange(self.z_range[0], self.z_range[1]+self.delta, self.delta):
            for x in np.arange(self.x_range[0], self.x_range[1]+self.delta, self.delta):
                for y in np.arange(self.y_range[0], self.y_range[1]+self.delta, self.delta):
                    pose = np.array([x,y,z])
                    
                    # trying to attmept muliple executions
                    for i in range(10):
                        execute = self.robot.arm.set_ee_pose(plan=True, position=pose, orientation=np.array(self.quat))
                        if execute:
                            break
                    
                    # paus efor some time before recording the data
                    time.sleep(self.pause_time)
                    
                    # marker data
                    # record ar marker pose
                    marker_pose = self.ar_marker.get_pose
                    if marker_pose is None:
                        colored("Ar marker is not visible into image\n", "red")
                        continue

                    trans, rot, quat = self.robot.arm.pose_ee
                    arm_pose = {'position':trans, 'orientation':rot}

                     # add data to list
                    arm_data_list.append(arm_pose)
                    marker_data_list.append(marker_pose)
                    count += 1
                    colored("Recorded {:04d} data\n\n".format(count), "green")

        # save data
        print("writing data to file\n")
        pickle.dump({'data':arm_data_list}, open( os.path.join(self.data_dir, 'arm.p'), "wb"), protocol=2)
        pickle.dump({'data':marker_data_list}, open( os.path.join(self.data_dir, 'marker.p'), "wb"), protocol=2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process args for data collection")
    parser.add_argument('--ar_id', help='Ar marker Id', type=int)
    parser.add_argument('--x_range', help='x range of end of effector [min, max]', nargs='+', default=[])
    parser.add_argument('--y_range', help='y range of end of effector [min, max]', nargs='+', default=[])
    parser.add_argument('--z_range', help='z range of end of effector [min, max]', nargs='+', default=[])
    parser.add_argument('--quat', help='quaternion [x,y,z,w]', nargs='+', default=[])
    parser.add_argument('--data_dir', help='Directory to store data points', type=str, default="robot_ar_data")
    parser.add_argument('--delta', help='delta in position', type=float, default=0.05)
    parser.add_argument('--pause_time', help='delta in position', type=float, default=0.05)

    args = parser.parse_args()
    data_collector = RobotArDataCollectorAuto(args)     

