// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include "azure_kinect_camera.h"

namespace rbgt {

AzureKinectCamera::~AzureKinectCamera() {
  if (initialized_) {
    device_.stop_cameras();
    device_.close();
  }
}

bool AzureKinectCamera::Init(const std::string &name) {
  name_ = name;

  if (!initialized_) {
    constexpr int kTimeoutInMs = 100;
    constexpr int kNumberImagesDropped = 10;

    // Configure color camera
    config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config_.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config_.color_resolution = K4A_COLOR_RESOLUTION_720P;

    // Check if camera is available
    if (k4a::device::get_installed_count() == 0) return false;

    // Start camera
    device_ = k4a::device::open(K4A_DEVICE_DEFAULT);
    device_.start_cameras(&config_);

    // Load multiple images to adjust to white balance
    for (int i = 0; i < kNumberImagesDropped; ++i) {
      while (!device_.get_capture(&capture_,
                                  std::chrono::milliseconds{kTimeoutInMs}))
        ;
    }
  }

  // Load intrinsics from camera
  const k4a_calibration_camera_t calibration{
      device_.get_calibration(config_.depth_mode, config_.color_resolution)
          .color_camera_calibration};
  const k4a_calibration_intrinsic_parameters_t::_param param =
      calibration.intrinsics.parameters.param;
  intrinsics_.fu = param.fx;
  intrinsics_.fv = param.fy;
  intrinsics_.ppu = param.cx;
  intrinsics_.ppv = param.cy;
  intrinsics_.width = calibration.resolution_width;
  intrinsics_.height = calibration.resolution_height;

  // Scale intrinsics acording to image scale
  intrinsics_.fu *= image_scale_;
  intrinsics_.fv *= image_scale_;

  // Calculate distortion map
  cv::Mat1f camera_matrix(3, 3);
  camera_matrix << param.fx, 0, param.cx, 0, param.fy, param.cy, 0, 0, 1;
  cv::Mat1f new_camera_matrix(3, 3);
  new_camera_matrix << intrinsics_.fu, 0, intrinsics_.ppu, 0, intrinsics_.fv,
      intrinsics_.ppv, 0, 0, 1;
  cv::Mat1f distortion_coeff(1, 8);
  distortion_coeff << param.k1, param.k2, param.p1, param.p2, param.k3,
      param.k4, param.k5, param.k6;
  cv::Mat map1, map2, map3;
  cv::initUndistortRectifyMap(
      camera_matrix, distortion_coeff, cv::Mat{}, new_camera_matrix,
      cv::Size{intrinsics_.width, intrinsics_.height}, CV_32FC1, map1, map2);
  cv::convertMaps(map1, map2, distortion_map_, map3, CV_16SC2, true);

  // Update image
  UpdateImage();
  initialized_ = true;
  return true;
}

void AzureKinectCamera::set_image_scale(float image_scale) {
  image_scale_ = image_scale;
}

bool AzureKinectCamera::UpdateImage() {
  // Get image
  constexpr int kTimeoutInMs = 33;
  device_.get_capture(&capture_, std::chrono::milliseconds{kTimeoutInMs});
  k4a::image k4a_image{capture_.get_color_image()};

  // Undistort image
  cv::Mat temp_image;
  cv::cvtColor(cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_8UC4,
                       (void *)k4a_image.get_buffer(), cv::Mat::AUTO_STEP},
               temp_image, cv::COLOR_RGBA2RGB);
  cv::remap(temp_image, image_, distortion_map_, cv::Mat(), cv::INTER_NEAREST,
            cv::BORDER_CONSTANT);

  return true;
}

}  // namespace rbgt
