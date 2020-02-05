# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from pyrobot import Robot
import hydra

@hydra.main(config_path="base_velocity_control.yaml")
def main(cfg):
    bot = Robot(cfg.botname, use_arm=False,
                base_config={'base_planner': cfg.base_planner,
                             'base_controller': cfg.base_controller})
    bot.base.set_vel(fwd_speed=cfg.linear_speed,
                     turn_speed=cfg.angular_speed,
                     exe_time=cfg.duration)
    bot.base.stop()


if __name__ == '__main__':
    main()
