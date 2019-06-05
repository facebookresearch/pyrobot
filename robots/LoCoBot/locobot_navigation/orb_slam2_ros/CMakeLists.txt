cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2_ros)

catkin_python_setup()

message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()


set(ORBSLAM2_LIBRARY_PATH "$ENV{ORBSLAM2_LIBRARY_PATH}")
if (ORBSLAM2_LIBRARY_PATH)
    message(STATUS "ORBSLAM2_LIBRARY_PATH at $ENV{ORBSLAM2_LIBRARY_PATH}")
else ()
    message(FATAL_ERROR "Undefined ORBSLAM2_LIBRARY_PATH environment variable.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${ORBSLAM2_LIBRARY_PATH}/cmake_modules)

find_package(OpenCV 3 REQUIRED)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        geometry_msgs
        image_transport
        message_filters
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
        message_generation
        )

add_message_files(
        FILES
        Traj.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs#   sensor_msgs#   std_msgs
)


catkin_package(
        CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters roscpp rospy sensor_msgs std_msgs tf message_runtime
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${ORBSLAM2_LIBRARY_PATH}
        ${ORBSLAM2_LIBRARY_PATH}/include
        ${PROJECT_SOURCE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${ORBSLAM2_LIBRARY_PATH}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${ORBSLAM2_LIBRARY_PATH}/Thirdparty/g2o/lib/libg2o.so
        ${ORBSLAM2_LIBRARY_PATH}/lib/libORB_SLAM2.so
        -lboost_system
        )

add_executable(orb_slam2_rgbd src/orb_ros_rgbd.cc)
add_dependencies(orb_slam2_rgbd orb_slam2_ros_generate_messages_cpp)
target_link_libraries(orb_slam2_rgbd
        ${catkin_LIBRARIES}
        ${LIBS}
        )
add_executable(orb_slam2_gen_cfg src/gen_cfg_file.cc)
add_dependencies(orb_slam2_gen_cfg orb_slam2_ros_generate_messages_cpp)
target_link_libraries(orb_slam2_gen_cfg
        ${catkin_LIBRARIES}
        ${LIBS}
        )
