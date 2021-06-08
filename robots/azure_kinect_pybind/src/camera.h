// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_CAMERA_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_CAMERA_H_

#include <string>
#include <opencv2/opencv.hpp>

namespace rbgt {

// Commonly used structs
using Intrinsics = struct Intrinsics {
  float fu, fv;
  float ppu, ppv;
  int width, height;
};

// Abstract class that defines a camera and functionality to save images.
// It is also able to hold a camera pose.
class Camera {
 public:
  // Getters
  const cv::Mat &image() const;
  const std::string &name() const;
  const Intrinsics &intrinsics() const;
  bool initialized() const;

 protected:
  // Variables and data
  cv::Mat image_;
  std::string name_{};
  Intrinsics intrinsics_{};

  // Internal state
  bool initialized_ = false;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_CAMERA_H_
