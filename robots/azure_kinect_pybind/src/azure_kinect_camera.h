// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_AZURE_KINECT_CAMERA_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_AZURE_KINECT_CAMERA_H_

#include "camera.h"

#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>

namespace rbgt {

// Class that allows to get new color images from an AzureKinect
class AzureKinectCamera : public Camera {
 public:
  // Constructor and initialization
  ~AzureKinectCamera();
  bool Init(const std::string &name);
  void set_image_scale(float image_scale);

  // Main method
  bool UpdateImage();

 private:
  k4a::device device_;
  k4a::capture capture_;
  k4a_device_configuration_t config_;
  float image_scale_ = 1.05f;
  cv::Mat distortion_map_;
};

}  // namespace rgbt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_AZURE_KINECT_CAMERA_H_
