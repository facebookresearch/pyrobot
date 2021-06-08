#include "src/camera.h"
#include "src/azure_kinect_camera.h"

#include "ndarray_converter.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <opencv2/opencv.hpp>

namespace py = pybind11;

PYBIND11_MODULE(azure_kinect_pybind, m) {
    NDArrayConverter::init_numpy();

    // -------------------------------------
    // Camera
    py::class_<rbgt::Camera, std::shared_ptr<rbgt::Camera>> cam(m, "Camera");
    cam.def(py::init<>());

    cam.def("image", &rbgt::Camera::image);
    cam.def("name", &rbgt::Camera::name);
    cam.def("intrinsics", &rbgt::Camera::intrinsics);
    cam.def("initialized", &rbgt::Camera::initialized);

    // -------------------------------------
    // AzureKinectCamera
    py::class_<rbgt::AzureKinectCamera, rbgt::Camera, std::shared_ptr<rbgt::AzureKinectCamera>> akc(m, "AzureKinectCamera");
    akc.def(py::init<>());
    akc.def("Init", &rbgt::AzureKinectCamera::Init);
    akc.def("set_image_scale", &rbgt::AzureKinectCamera::set_image_scale);
    akc.def("UpdateImage", &rbgt::AzureKinectCamera::UpdateImage);

    // -------------------------------------
    // intrinstics
    py::class_<rbgt::Intrinsics> intrinsitcs(m, "Intrinsics");
    intrinsitcs.def(py::init<>());
    intrinsitcs.def_readwrite("fu", &rbgt::Intrinsics::fu);
    intrinsitcs.def_readwrite("fv", &rbgt::Intrinsics::fv);
    intrinsitcs.def_readwrite("ppu", &rbgt::Intrinsics::ppu);
    intrinsitcs.def_readwrite("ppv", &rbgt::Intrinsics::ppv);
    intrinsitcs.def_readwrite("width", &rbgt::Intrinsics::width);
    intrinsitcs.def_readwrite("height", &rbgt::Intrinsics::height);
}

