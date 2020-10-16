# franka_tools

Some helper classes for controlling and handling the Franka Emika Panda robot.

### FrankaControllerManagerInterface

- List, start, stop, load available controllers for the robot
- Get the current controller status (commands, set points, controller gains, etc.)
- Update controller parameters through *ControllerParamConfigClient* (see below)

### FrankaFramesInterface

- Get and Set end-effector frame and stiffness frame of the robot easily
- Set the frames to known frames (such as links on the robot) directly
 
### ControllerParamConfigClient

- Get and set the controller parameters (gains) for the active controller

### JointTrajectoryActionClient

- Command robot to given joint position(s) smoothly. (Uses the FollowJointTrajectory service from ROS *control_msgs* package)
- Smoothly move to a desired (valid) pose without having to interpolate for smoothness (trajectory interpolation done internally)

### CollisionBehaviourInterface

- Define collision and contact thresholds for the robot safety and contact detection.