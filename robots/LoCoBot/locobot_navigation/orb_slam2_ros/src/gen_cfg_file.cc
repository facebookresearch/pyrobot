/*******************************************************************************
Copyright (c) Facebook, Inc. and its affiliates.

This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.

*******************************************************************************/

#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include <vector>
#include <boost/array.hpp>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <chrono>
class Camera {
 public:
  Camera(std::string filename, std::string camInfoTopic,
         float camBaseline, bool bRGB, ros::NodeHandle &n) : mn(n),
                                                             mbGotCamInfo(false),
                                                             msFilename(filename),
                                                             mfCamBaseline(camBaseline) {
    mnRGB = bRGB;
    mCamInfoSub = mn.subscribe(camInfoTopic,
                               1, &Camera::caminfoCallback, this);
  }

  void caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    mvD = msg->D;
    maK = msg->K;
    mnCamHeight = msg->height;
    mnCamWidth = msg->width;
    mbGotCamInfo = true;
  }

  void run() {
    ros::Rate r(10);
    auto start = std::chrono::system_clock::now();
    while (1) {
      ros::spinOnce();
      if (mbGotCamInfo) {
        cv::FileStorage fs(msFilename, cv::FileStorage::WRITE);
        fs << "Camera_fx" << maK[0];
        fs << "Camera_fy" << maK[4];
        fs << "Camera_cx" << maK[2];
        fs << "Camera_cy" << maK[5];
        fs << "Camera_k1" << mvD[0];
        fs << "Camera_k2" << mvD[1];
        fs << "Camera_p1" << mvD[2];
        fs << "Camera_p2" << mvD[3];
        fs << "Camera_k3" << mvD[4];
        fs << "Camera_width" << mnCamWidth;
        fs << "Camera_height" << mnCamHeight;
        fs << "Camera_fps" << 30.0;
        float bf = mfCamBaseline / 1000.0 * maK[0];
        fs << "Camera_bf" << bf;
        fs << "Camera_RGB" << mnRGB;
        fs << "ThDepth" << 40.0;
        fs << "DepthMapFactor" << 1000.0;
        fs << "ORBextractor_nFeatures" << 1000;
        fs << "ORBextractor_scaleFactor" << 1.2;
        fs << "ORBextractor_nLevels" << 8;
        fs << "ORBextractor_iniThFAST" << 20;
        fs << "ORBextractor_minThFAST" << 7;
        fs << "Viewer_KeyFrameSize" << 0.05;
        fs << "Viewer_KeyFrameLineWidth" << 1;
        fs << "Viewer_GraphLineWidth" << 0.9;
        fs << "Viewer_PointSize" << 2;
        fs << "Viewer_CameraSize" << 0.08;
        fs << "Viewer_CameraLineWidth" << 3;
        fs << "Viewer_ViewpointX" << 0;
        fs << "Viewer_ViewpointY" << -0.7;
        fs << "Viewer_ViewpointZ" << -1.8;
        fs << "Viewer_ViewpointF" << 500;
        fs << "DepthImgSavePath" << "./Imgs/DepthImgs";
        fs << "RGBImgSavePath" << "./Imgs/RGBImgs";
        fs.release();
        std::cout << "Camera configuration file generated." << std::endl;
        boost::filesystem::path pDir(msFilename);
        boost::filesystem::path script("bash_scripts/replace_underscore.sh");
        boost::filesystem::path scriptPath = pDir.parent_path().parent_path() / script;
        std::cout << "Running script:" << scriptPath.string() << std::endl;
        system(scriptPath.string().c_str());
        break;
      }
      r.sleep();
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = end - start;
      if (diff.count() > 10) {
        std::cout << "******************************************" << std::endl;
        std::cout << "Camera configuration file not generated!!!" << std::endl;
        std::cout << "Please manually generate it" << std::endl;
        std::cout << "******************************************" << std::endl;
        break;
      }
    }
  }

  ros::NodeHandle mn;
  ros::Subscriber mCamInfoSub;
  std::string msFilename;
  int mnCamHeight;
  int mnCamWidth;
  int mnRGB;
  float mfCamBaseline;
  std::vector<double> mvD;
  boost::array<double, 9> maK;
  bool mbGotCamInfo;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gen_cfg_file");
  ros::start();
  ros::NodeHandle nh("~");

  std::string filename;
  std::string camInfoTopic;
  float camBaseline;
  bool bRGB;
  nh.getParam("filename", filename);
  nh.getParam("camInfoTopic", camInfoTopic);
  nh.getParam("camBaseline", camBaseline);
  nh.getParam("bRGB", bRGB);
  Camera cam(filename, camInfoTopic,
             camBaseline, bRGB, nh);
  cam.run();
  return 0;
}

