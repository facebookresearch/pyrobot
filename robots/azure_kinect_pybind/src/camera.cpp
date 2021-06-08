// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include "camera.h"

namespace rbgt {

const cv::Mat &Camera::image() const { return image_; }

const std::string &Camera::name() const { return name_; }

const Intrinsics &Camera::intrinsics() const { return intrinsics_; }

bool Camera::initialized() const { return initialized_; }

}  // namespace rbgt
