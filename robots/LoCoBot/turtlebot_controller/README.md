turlebot_controller
===================================================

Turtlebot SE(2) non-holonomic base controller.

Prerequisites
------

- Install [ROS](http://wiki.ros.org/Distributions)
- Install turtlebot dependencies
  
  ```bash
  sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs ros-indigo-turtlebot-rviz-launchers
  ```

Installation
------

- Initialize a catkin workspace (if you are using an existing catkin workspace this step is not needed)
    
  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  catkin_init_workspace
  ```

  Before running setup the environment variables

  ```bash
  source ~/catkin_ws/devel/setup.bash
  ```

- Clone this repository in ```~/catkin_ws/src```

  ```bash
  git clone https://<username>@bitbucket.org/gtrll/turtlebot_controller.git
  ```

- To compile
    
  ```bash
  catkin_make
  ```

Usage
------

- To run gazebo simulator for turtlebot and the controller

  ```bash
  roslaunch turtlebot_controller turtlebot.launch
  ```

- To run just the controller

  ```bash
  roslaunch turtlebot_controller turtlebot_controller_node.launch
  ```
