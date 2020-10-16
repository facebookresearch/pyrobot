# franka_ros_controllers

Controllers for the Franka Emika Panda robot using ROS topics.

### Features:

- Includes joint position, joint velocity, and joint effort (direct torque, indirect position and impedance) controllers that can be controlled directly through ROS topic 
- Same topic is used for all controllers; different keyword required in the ROS message for each controller (see *set_joint_positions*, *set_joint_velocities*, etc implemented in *franka_ros_interface/franka_interface/arm.py*)
- Controller gains and other parameters can be controlled using dynamic reconfiguration or service calls (or using python API: *ControllerParamConfigClient* from *franka_ros_interface/franka_tools*). Default values can be set in the config file.

