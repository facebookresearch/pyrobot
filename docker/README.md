# AIRobot Docker Instructions

- [Dependency Installation](#dependencies)
- [Pull the Pre-Built Image](#pulling-the-pre-built-images)
- [Usage](#usage)
- [Build the Image Locally](#build-the-image-locally)

### Dependencies
System dependencies:
- Linux OS (tested on Ubuntu 18.04)
- Nvidia GPU
- Nvidia driver version >= 470.42

#### Local Docker engine setup
Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) (you may want to configure your user permissions so you don't have to use sudo with every docker command -- see [here](https://docs.docker.com/install/linux/linux-postinstall/))
```
sudo apt-get remove docker docker-engine docker.io containerd runc
sudo apt-get update
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Install [nvidia-container-runtime](https://github.com/nvidia/nvidia-container-runtime#docker-engine-setup) (main instructions from the link shown below)

```
curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
sudo apt-get update
sudo apt-get install nvidia-container-runtime
```
Follow the steps for creating a drop in file to register the runtime with docker

```
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo tee /etc/systemd/system/docker.service.d/override.conf <<EOF
[Service]
ExecStart=
ExecStart=/usr/bin/dockerd --host=fd:// --add-runtime=nvidia=/usr/bin/nvidia-container-runtime
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker
```

## Pulling the Pre-Built Images
The image is set up with CUDA 11.4, and PyTorch 1.10 installed.
```
docker pull improbableailab/ur5e-ros:kinetic-cuda
```


## Usage
### Running the container
Open two terminals

#### Terminal 1
In the first terminal, from within this directory (```/path/to/airobot/docker/```), run the ```run_image.bash``` script to launch the container. You will enter a terminal session from within the container after launching.
```
./run_image.bash
```

#### Termainal 2
From the second terminal, run the following bash script to allow the container access to your local X server for visualization (there are better ways to do this, but we keep this relatively secure by only providing access to xhost to the specific system we want)

```
./visualize_access.bash
```

You can now run programs from within the container that are GUI-based, and have the GUI appear
on your host screen (i.e. Gazebo, RViz, PyBullet)

First, enter the container by starting a new interactive terminal session (```$CONTAINER_ID``` can usually be found by tab completing the below command, or viewing the information about the running containers by running ```docker ps```) and source the catkin workspace that has been built with the following commands
```
docker exec -it $CONTAINER_ID bash
```

### Run the simulated UR robot in Gazebo

Then, for instance, you can launch a simulated UR5e in Gazebo, and start MoveIt! with it, with the following command

```
roslaunch ur5e_bringup ur5e_start.launch sim:=true
```

Or you can launch a simulated UR5e in PyBullet with ```airobot``` with the following commands
```
cd /home/improbable/airobot/examples/ur5e/sim/
python joint_position_control.py
```

### Run the real UR robot

You can also connect to the real robot and control it via the ```airobot``` API functions (```<robot_ip>``` can be viewed from the PolyScope interface on the real robot's teach pendant. First bringup the connection to the robot with the following command, after connecting to the local network via ethernet cable (more instructions avaiable [here](https://github.com/Improbable-AI/ur5e)).

```
roslaunch ur5e_bringup ur5e_start.launch robot_ip:=<robot_ip>
```
Then run the following command to run the ```joint_position_control``` example (from a new terminal, either using ```tmux``` or by entering the container from a new local terminal with the same ```docker exec``` command above)
```
cd /home/improbable/airobot/examples/ur5e/real/
python joint_position_control.py
```

## Build the Image Locally
From within this directory (```/path/to/airobot/docker/```), run the following command to build the image. This will take quite a bit of time if you have not done it locally before. Please see the ```docker_build.py``` script for additional arguments that can be passed to the build command (i.e. image name, which file to use, dry run, etc.)
```
python docker_build.py
```

If you want to use a different dockerfile than the specified default, run
```
python docker_build.py -f /path/to/dockerfile.dockerfile
```

(if you are building the image locally you will need to set up your machine with public keys linked to your github account for cloning private repositories required during building)
