# Examples for using PyRobot API to control Azure Kinnect

This tutorials assumes that you have installed PyRobot already using the official instructions.

## Note
This tutorial has been tested to work with the ROS-Kinectic and Ubuntu 16.04


## Setting up the Azure Kinect software

1. Clone the Azure Kinect SDK
```bash
git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
cd Azure-Kinect-Sensor-SDK
git checkout release/1.1.x
```

2. Install dependencies using the provided script. 
```bash
bash ./scripts/bootstrap-ubuntu.sh
```

3. Source PyRobot virtual environment and install latest Cmake
```bash
load_pyrobot_env # this alias must be already created when PyRobot is installed
pip install cmake
```

4. Build and install Azure Kinect SDK inside the virtual environment
```bash
sudo apt-get install ninja-build
mkdir build && cd build
cmake .. -GNinja
ninja
sudo ninja install
```

5. Download the missing ```libdepthengine.so.1.0``` and ```libstdc++.so.6``` from [here](https://drive.google.com/drive/folders/1PYci8STgGtf2GMMl1I8wrVRMXMw-mPbC?usp=sharing) into ```path/to/Azure-Kinect-Sensor-SDK/build/bin```

6. Test out that the setup is working connecting the Azure kinect to the computer and running
```bash
sudo path/to/Azure-Kinect-Sensor-SDK/build/bin/k4aviewer
```

Now, lets install the Azure Kinect ROS packages,

7. Setup Azure Kinect catkin workspace,
```bash
mkdir -p azure_kinect_ws/src
cd azure_kinect_ws/src
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
cd Azure_kinect_ROS_Driver
git checkout be9a528ddac3f9a494045b7acd76b7b32bd17105
```

8. Make minor edits to the codebase. If you were to build the workspace now, you would get errors relating to std::atomic syntax.

To fix this, open ```<repo>/include/azure_kinect_ros_driver/k4a_ros_device.h and convert all instances of std::atomic_TYPE type declarations to std::atomic<TYPE>```. Below is a diff of the edits I made.
```
@@ -117,11 +117,11 @@ class K4AROSDevice
  volatile bool running_;
 
  // Last capture timestamp for synchronizing playback capture and imu thread
       -    std::atomic_int64_t last_capture_time_usec_;
       +    std::atomic<int64_t> last_capture_time_usec_;
 
  // Last imu timestamp for synchronizing playback capture and imu thread
       -    std::atomic_uint64_t last_imu_time_usec_;
       -    std::atomic_bool imu_stream_end_of_file_;
       +    std::atomic<uint64_t> last_imu_time_usec_;
       +    std::atomic<bool> imu_stream_end_of_file_;
 
  // Threads
  std::thread frame_publisher_thread_;

```

9. Compile the ROS packages -
```bash
cd path/to/azure_kinect_ws
catkin_make
```

10. Download the missing ```libdepthengine.so.1.0``` and ```libstdc++.so.6``` from [here](https://drive.google.com/drive/folders/1PYci8STgGtf2GMMl1I8wrVRMXMw-mPbC?usp=sharing) into ```path/to/azure_kinect_ws/devel/lib```


11. Copy udev rules from the ROS driver repo to your machine. 
```bash
sudo cd path/to/Azure-Kinect-Sensor-SDK/scripts/99-k4a.rules /etc/udev/rules.d/
```
**Unplug and replug your sensor into the machine after copying the file over.**

12. Finally, test out the ROS-Kinect setup by running,
```bash
source path/to/azure_kinect_ws/devel/setup.bash
roslaunch azure_kinect_ros_driver driver.launch
```


## Running the examples

1st terminal,
```bash
source path/to/azure_kinect_ws/devel/setup.bash
roslaunch azure_kinect_ros_driver driver.launch
```

2nd terminal,
```bash
load_pyenv_pyrobot 
python azure_kinect_example.py
```

