cmake_minimum_required(VERSION 2.8.3)
project(franka_interface)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  franka_control
  controller_manager
  franka_core_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)


catkin_package(
  LIBRARIES custom_franka_state_controller
  CATKIN_DEPENDS
    controller_interface
    franka_msgs
    dynamic_reconfigure
    franka_hw
    franka_control
    franka_core_msgs
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

add_library(custom_franka_state_controller
  src/robot_state_controller.cpp
)

add_dependencies(custom_franka_state_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(custom_franka_state_controller PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(custom_franka_state_controller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(custom_franka_state_controller PUBLIC
  include
)

## franka_control_node
add_executable(custom_franka_control_node
  src/franka_control_node.cpp
  src/motion_controller_interface.cpp
)

add_dependencies(custom_franka_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}

)

target_link_libraries(custom_franka_control_node
  ${Franka_LIBRARIES}
  ${franka_control_LIBRARIES}
  # franka_control_services
  ${catkin_LIBRARIES}
)

target_include_directories(custom_franka_control_node SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

## Installation
install(TARGETS custom_franka_state_controller
                custom_franka_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES state_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(custom_franka_state_controller FILES ${SOURCES} ${HEADERS})
  add_tidy_target(custom_franka_state_controller custom_franka_control_node
    FILES ${SOURCES}
    DEPENDS custom_franka_state_controller
  )
endif()


catkin_python_setup()
# ## Tools
# include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
#   RESULT_VARIABLE CLANG_TOOLS
# )
# if(CLANG_TOOLS)
#   file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
#   file(GLOB_RECURSE HEADERS
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
#   )
#   add_format_target(custom_franka_state_controller FILES ${SOURCES} ${HEADERS})
#   add_tidy_target(custom_franka_state_controller
#     FILES ${SOURCES}
#     DEPENDS custom_franka_state_controller
#   )
# endif()