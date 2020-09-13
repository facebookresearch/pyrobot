
#  Please use this file for debugging while re-factoring PyRobot

import hydra

from hydra.experimental import initialize, compose
from omegaconf import DictConfig

import numpy as np

# @hydra.main(config_path="hydra_config/default_config.yaml")
# def main(hydra_cfg):
# 	print(hydra_cfg.pretty())

# 	assert type(hydra_cfg) == DictConfig
# 	robot0 = hydra_cfg.environment.robots[0]
# 	print(robot0)

# run: roslaunch locobot_control main.launch use_sim:=true

from pyrobot import World
def main():
	
	world =  World(config_name='env/simple_env.yaml')

	bot = world.robots['locobot']

	bot["base"].set_vel(1.0,1.0,2.0)



if __name__ == "__main__":
    main()