# automatiing the data collection for camera calibration
import os
from IPython import embed
from pyrobot import Robot
import time
import pickle
import argparse
from util import ArMarker
from termcolor import colored

class RobotArDataCollectorManual(object):

    def __init__(self, args):
        self.robot = Robot('sawyer',use_base=False,use_camera=False)
        self.ar_marker = ArMarker(args.ar_id)
        self.num_data_points = args.num_data

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
        i = 0
        while i < self.num_data_points:
            input = raw_input(colored("press r to record data or e to exit data collection\n", "green"))
            if input == 'e':
                colored("exiting data collection\n", "red")
                break
            elif input == 'r':
                # record arm pose
                trans, rot, quat = self.robot.arm.pose_ee
                arm_pose = {'position':trans, 'orientation':rot}

                # record ar marker pose
                marker_pose = self.ar_marker.get_pose
                if marker_pose is None:
                    colored("Ar marker is not visible into image, move arm so that marker is visible in image frame\n", "red")
                    continue

                # add data to list
                arm_data_list.append(arm_pose)
                marker_data_list.append(marker_pose)
                i += 1
                colored("Recorded {:02d} data\n\n".format(i), "green")

        # save data
        print("writing data to file\n")
        pickle.dump({'data':arm_data_list}, open( os.path.join(self.data_dir, 'arm.p'), "wb"), protocol=2)
        pickle.dump({'data':marker_data_list}, open( os.path.join(self.data_dir, 'marker.p'),"wb"), protocol=2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process args for data collection")
    parser.add_argument('--ar_id', help='Ar marker Id', type=int)
    parser.add_argument('--num_data', help='Num of data points', type=int, default=20)
    parser.add_argument('--data_dir', help='Directory to store data points', type=str, default="robot_ar_data")

    args = parser.parse_args()
    data_collector = RobotArDataCollectorManual(args)      
