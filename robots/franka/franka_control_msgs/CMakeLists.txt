cmake_minimum_required(VERSION 2.8.3)

project(franka_control_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        actionlib_msgs
        std_msgs
        franka_msgs
        geometry_msgs
        sensor_msgs
        control_msgs
)

add_message_files( DIRECTORY msg
        FILES
        JointCommand.msg
        RobotState.msg
        EndPointState.msg
        JointLimits.msg
        JointControllerStates.msg
)

# add_action_files(DIRECTORY action FILES CalibrationCommand.action)

## Build
generate_messages(DEPENDENCIES std_msgs geometry_msgs control_msgs sensor_msgs actionlib_msgs franka_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs control_msgs sensor_msgs franka_msgs actionlib_msgs)


## Install


## Testing
