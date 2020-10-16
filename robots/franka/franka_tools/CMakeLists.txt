cmake_minimum_required(VERSION 2.8.3)
project(franka_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  franka_control
  franka_interface
  franka_core_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_python_setup()



# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


