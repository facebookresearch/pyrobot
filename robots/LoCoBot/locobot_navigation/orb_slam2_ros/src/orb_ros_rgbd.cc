/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <tuple>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include "orb_slam2_ros/Traj.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "Converter.h"
#include "System.h"
#include <thread>
#include <chrono>

using namespace std;

bool ContainNan(vector<float> vec) {
  for (auto it = vec.begin(); it != vec.end(); ++it) {
    if (isnan(*it)) {
      return true;
    }
  }
  return false;
}

class ImageGrabber {
 public:
  ImageGrabber(ORB_SLAM2::System *pSLAM, ros::NodeHandle &n) : mpSLAM(pSLAM), mn(n) {
    mPosePub = mn.advertise<geometry_msgs::PoseStamped>("slam/camera_pose", 10);
    mTrajPub = mn.advertise<orb_slam2_ros::Traj>("slam/camera_traj", 10);
  }

  void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);
  void PublishPose(cv::Mat Tcw);
  void PublishTraj(std::tuple<vector<int>, vector<vector<float> >, vector<vector<float> > > traj);

  ORB_SLAM2::System *mpSLAM;
  ros::NodeHandle mn;
  ros::Publisher mPosePub;
  ros::Publisher mTrajPub;
};

int main(int argc, char **argv) {
  // sleep for some time here because realsense node and orb_slam2_ros node
  // start together from a launch file leads to socket error sometimes

  std::this_thread::sleep_for(std::chrono::seconds(2));
  ros::init(argc, argv, "orb_slam2_rgbd");
  ros::start();
  ros::NodeHandle nh("~");

  std::string strVocFile;
  std::string strSettingsFile;
  std::string rgbTopic;
  std::string depthTopic;
  nh.getParam("voc_file", strVocFile);
  nh.getParam("settings_file", strSettingsFile);
  nh.getParam("rgb_topic", rgbTopic);
  nh.getParam("depth_topic", depthTopic);

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(strVocFile, strSettingsFile, ORB_SLAM2::System::RGBD, true);
  ImageGrabber igb(&SLAM, nh);

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgbTopic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depthTopic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));
  ros::spin();

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
  std::tuple<vector<int>, vector<vector<float> >, vector<vector<float> > > traj =
      mpSLAM->GetKeyFrameTrajectory();
  PublishTraj(traj);
  PublishPose(Tcw);
}

void ImageGrabber::PublishPose(cv::Mat Tcw) {
  geometry_msgs::PoseStamped poseMSG;
  if (!Tcw.empty()) {
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    poseMSG.pose.position.x = twc.at<float>(0);
    poseMSG.pose.position.y = twc.at<float>(1);
    poseMSG.pose.position.z = twc.at<float>(2);
    poseMSG.pose.orientation.x = q[0];
    poseMSG.pose.orientation.y = q[1];
    poseMSG.pose.orientation.z = q[2];
    poseMSG.pose.orientation.w = q[3];
    poseMSG.header.frame_id = "slam_camera_color_frame";
    poseMSG.header.stamp = ros::Time::now();
    mPosePub.publish(poseMSG);
  }
}

void ImageGrabber::PublishTraj(std::tuple<vector<int>,
                                          vector<vector<float> >,
                                          vector<vector<float> > > traj) {
  orb_slam2_ros::Traj TrajMsg;
  TrajMsg.header.frame_id = "slam_camera_color_frame";
  TrajMsg.header.stamp = ros::Time::now();
  vector<int> vnKFids = std::get<0>(traj);
  vector<vector<float>> vvfKFpos = std::get<1>(traj);
  vector<vector<float>> vvfKFquat = std::get<2>(traj);
  for (size_t i = 0; i < vnKFids.size(); i++) {
    int KFid = vnKFids[i];
    vector<float> vfKFpos = vvfKFpos[i];
    vector<float> vfKFquat = vvfKFquat[i];
    if (ContainNan(vfKFpos) || ContainNan(vfKFquat)) {
      continue;
    }
    geometry_msgs::Pose tmpPose;
    tmpPose.position.x = vfKFpos[0];
    tmpPose.position.y = vfKFpos[1];
    tmpPose.position.z = vfKFpos[2];
    tmpPose.orientation.x = vfKFquat[0];
    tmpPose.orientation.y = vfKFquat[1];
    tmpPose.orientation.z = vfKFquat[2];
    tmpPose.orientation.w = vfKFquat[3];
    TrajMsg.mnIds.push_back(KFid);
    TrajMsg.poses.push_back(tmpPose);
  }
  mTrajPub.publish(TrajMsg);
}
