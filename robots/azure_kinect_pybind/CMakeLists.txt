cmake_minimum_required(VERSION 3.10)

# Azure Kinect Python Binding
# =============================================================================
project(azure_kinect_pybind)

# Libraries
# =============================================================================
find_package(k4a REQUIRED)
find_package(OpenCV 3.0.0 REQUIRED)
find_package(Eigen3 3.3.2 REQUIRED)


# Compiler Settings
# =============================================================================

# Default settings
set(CMAKE_CXX_STANDARD 17)

# Directories
# =============================================================================

# Include Directories
include_directories("${CMAKE_SOURCE_DIR}/third_party")

# Sub Directories
# add_subdirectory(src)

file (GLOB SOURCE_FILES "src/*.cpp" ndarray_converter.cpp)
file (GLOB HEADER_FILES "src/*.h" ndarray_converter.h)
file (GLOB PYTHON_FILES "azure_kinect_pybind.cpp")

set(LIBRARIES ${OpenCV_LIBRARIES} pthread stdc++fs Eigen3::Eigen k4a::k4a)

add_subdirectory(pybind11)
pybind11_add_module(azure_kinect_pybind ${SOURCE_FILES}
								${HEADER_FILES}
								${PYTHON_FILES})
target_link_libraries(azure_kinect_pybind PUBLIC ${LIBRARIES})
