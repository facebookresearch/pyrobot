# franka_panda_desctiption 

Robot description package modified from [*franka_ros*](https://frankaemika.github.io/docs/franka_ros.html) package to include dynamics parameters for the robot arm (as estimated in [this paper](https://hal.inria.fr/hal-02265293/document)). Also includes transmission and control definitions required for Gazebo support (see [*panda_simulator*](https://github.com/justagist/panda_simulator) package).

## Related Packages
- [*panda_simulator*](https://github.com/justagist/panda_simulator) : Simulation in Gazebo with exposed controllers and state feedback using ROS topics and services. The simulated robot uses the same ROS topics and services as the real robot when using the [*franka_ros_interface*](https://github.com/justagist/franka_ros_interface).
- [*franka_ros_interface*](https://github.com/justagist/franka_ros_interface) : A ROS API for controlling and managing the Franka Emika Panda robot (real and simulated). Contains controllers for the robot (joint position, velocity, torque), interfaces for the gripper, controller manager, coordinate frames interface, etc.. Provides almost complete sim-to-real transfer of code.
- [*panda_robot*](https://github.com/justagist/panda_robot) : Python interface providing higher-level control of the robot integrated with its gripper, controller manager, coordinate frames manager, etc. It also provides access to the kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl).
