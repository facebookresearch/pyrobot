source /opt/ros/noetic/setup.bash
source /root/create_ws/devel/setup.bash
cd ~

mkdir -p locobot_ws/src
cd locobot_ws/src
git clone https://github.com/yujinrobot/yocs_msgs.git
git clone https://github.com/Improbable-AI/ar_track_alvar.git -b noetic-devel
git clone https://github.com/yujinrobot/yujin_ocs.git
git clone https://github.com/yujinrobot/kobuki.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_interactions.git
git clone https://github.com/turtlebot/turtlebot_simulator.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
git clone https://github.com/AutonomyLab/create_robot.git
git clone https://github.com/Improbable-AI/pyrobot.git
cd ..
rosdep update
apt-get update
rosdep install --from-paths src/create_robot src/dynamixel-workbench src/dynamixel-workbench-msgs src/ar_track_alvar -i -y
catkin build