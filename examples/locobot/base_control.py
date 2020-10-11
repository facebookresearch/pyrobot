
import hydra

from hydra.experimental import initialize, compose
from omegaconf import DictConfig

import numpy as np

from pyrobot import World, make_algorithm
from pyrobot.algorithms.kinematics import Kinematics 

import copy


def main():
    
    world = World(config_name='env/base_env.yaml')
    target_position = [1,1,0.5]

    # world.algorithms['movebase_control'].go_to_absolute(target_position, smooth=False) 
    world.algorithms['movebase_planner'].get_plan_absolute(1, 1, 0.5) 


if __name__ == "__main__":
    main()
