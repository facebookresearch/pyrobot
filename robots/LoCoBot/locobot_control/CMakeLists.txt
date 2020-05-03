################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(locobot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  actionlib
  actionlib_msgs
  message_generation
)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_service_files(
  FILES
  JointCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs dynamixel_workbench_msgs dynamixel_workbench_toolbox actionlib actionlib_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# add_executable(locobot_control src/locobot_control.cpp)
# add_dependencies(locobot_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(locobot_control ${catkin_LIBRARIES})

add_executable(locobot_controller src/locobot_controller.cpp)
add_dependencies(locobot_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(locobot_controller ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
#################################################################################
## Install
#################################################################################
# install(TARGETS locobot_control
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS locobot_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS nodes/robot_teleop_server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS nodes/keyboard_teleop_client.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS nodes/pointrobot3factor_ros_server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
################################################################################
# Test
################################################################################
