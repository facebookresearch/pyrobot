cd ~
mkdir -p create_ws/src
cd create_ws
catkin init
cd src
git clone https://github.com/AutonomyLab/libcreate.git
catkin build
