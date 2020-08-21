# Examples for using PyRObot to Control LoCoBot in V-Rep Simulator


## Getting Started

This tutorials assumes that you have already installed PyRobot and its virtual environment.

Please download and extract V-REP 3.6.2 for Ubuntu 16.04

Clone the V-Rep Python (PyRep) - git clone https://github.com/kalyanvasudev/PyRep.git

Add the following to your *~/.bashrc* file: (__NOTE__: the 'EDIT ME' in the first line)

```bash
export VREP_ROOT=EDIT/ME/PATH/TO/V-REP/INSTALL/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$VREP_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$VREP_ROOT
```

Now load the PyRobot virtual environment
```bash
source ~/pyenv_pyrobot_python3/bin/activate
```

Then go into the PyRep root folder ```cd PyRep``` and install PyRep
```bash
pip3 install -r requirements.txt
python3 setup.py install
```

## Resources 

For more information on how to use V-Rep simulator, users are highly encouraged to go through V-Rep [tutorials](http://www.coppeliarobotics.com/helpFiles/en/tutorials.htm). Particularly the BubbleRob [tutorial](http://www.coppeliarobotics.com/helpFiles/en/bubbleRobTutorial.htm)

User also encouraged to go through [PyRep](https://github.com/stepjam/PyRep) code for more information on the v-Rep python interface.
