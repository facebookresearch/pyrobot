
# Examples for using LoCoBot through remote control


## Getting started

* Prepare a client machine, and a LoCoBot (Server) with PyRobot installed. Make sure they connect to the same network.
* Copy the robot instance wrapper (`remote_locobot.py`) to LoCoBot
* Pip install `Pyro4` on both machine. 

## Launch the robot driver

* Turn on the robot
* Open a terminal, source the python virtual environment and start the launch file
```bash
load_pyrobot_env
roslaunch locobot_control main.launch
```

## Create server on the robot

* Open a new terminal, source the python virtual environment with `Pyro4` installed.  

```bash
load_pyrobot_env
```

* Start a server

```bash
python -m Pyro4.naming -n <Robot_IP>
```

* Open a new terminal, source the python virtual environment with `Pyro4` installed, and run the wrapper file

```bash
load_pyrobot_env
python remote_locobot.py --ip <Robot_IP>
```

## Example Usage

* On the client machine, open a new terminal, source the python virtual environment with `Pyro4` installed.  

```bash
load_pyrobot_env
```

* Instantiate one or more remote locobot instances

```bash
import Pyro4

bot1 = Pyro4.Proxy("PYRONAME:remotelocobot@<Robot_IP1>")
bot2 = Pyro4.Proxy("PYRONAME:remotelocobot@<Robot_IP2>")

pan_angle = bot1.get_pan()
tilt_angle = bot1.get_tilt()
bot1.set_pan_tilt(pan_angle + 0.1, tilt_angle - 0.1)

bot2.get_rgb()
bot2.get_depth()
```

Note that depth runs ~1.25 fps, and rgb runs ~0.25 fps on remote.