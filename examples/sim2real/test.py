from __future__ import print_function

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# the code is from on the publicly available implementation of the TD3 algorithm
# https://github.com/sfujim/TD3

import argparse
import os
import sys

import gym
import numpy as np
import torch

import TD3


# Runs policy for X episodes and returns average reward
def evaluate_policy(policy, eval_episodes=100):
    avg_reward = 0.0
    for _ in xrange(eval_episodes):
        obs = env.reset()
        done = False
        step_count = 0
        while not done:
            action = policy.select_action(np.array(obs))
            obs, reward, done, _ = env.step(action)
            avg_reward += reward
            step_count += 1
            print("step_count = {}".format(step_count))

        print("----------")
        print("Eposide ended")
        if done:
            print("Arm is within {} m to goal".format(0.01))
        else:
            print("Arm is within {} m to goal".format(-reward))
        print("----------")

    avg_reward /= eval_episodes

    print("---------------------------------------")
    print("Evaluation over %d episodes: %f" % (eval_episodes, avg_reward))
    print("---------------------------------------")
    return avg_reward


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--env_name", default="LocoBotEnv-v0"
    )  # OpenAI gym environment name
    parser.add_argument(
        "--seed", default=0, type=int
    )  # Sets Gym, PyTorch and Numpy seeds
    parser.add_argument(
        "--eval_episodes", default=1e2, type=float
    )  # How often (time steps) we evaluate
    parser.add_argument(
        "--valid_goals", action="store_true"
    )  # Frequency of delayed policy updates
    parser.add_argument("--use_real_robot", action="store_true")
    parser.add_argument("--file_name", type=str)  # Frequency of delayed policy updates
    parser.add_argument("--directory", type=str)  # Frequency of delayed policy updates

    args = parser.parse_args()

    root_path = "/".join(i for i in os.path.abspath(__file__).split("/")[:-1])
    sys.path.insert(0, root_path)
    import envs

    env = gym.make(args.env_name)
    env._renders = True
    if args.valid_goals:
        env._valid_goal = True
    if args.use_real_robot:
        env._use_real_robot = True
        env._max_episode_steps = 10

    # Set seeds
    env.seed(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    max_action = float(env.action_space.high[0])

    # Initialize policy
    policy = TD3.TD3(state_dim, action_dim, max_action)
    device = None if torch.cuda.is_available() else "cpu"
    policy.load(args.file_name, args.directory, map_location=device)
    evaluate_policy(policy)
