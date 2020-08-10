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
import utils


# Runs policy for X episodes and returns average reward
def evaluate_policy(policy, eval_episodes=100):
    avg_reward = 0.0
    for _ in xrange(eval_episodes):
        obs = env.reset()
        done = False
        while not done:
            action = policy.select_action(np.array(obs))
            obs, reward, done, _ = env.step(action)
            avg_reward += reward

    avg_reward /= eval_episodes

    print("---------------------------------------")
    print("Evaluation over {} episodes: {}".format(eval_episodes, avg_reward))
    print("---------------------------------------")
    return avg_reward


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--exp_name", type=str)  # Place to store log
    parser.add_argument(
        "--env_name", default="LocoBotEnv-v0"
    )  # OpenAI gym environment name
    parser.add_argument(
        "--seed", default=0, type=int
    )  # Sets Gym, PyTorch and Numpy seeds
    parser.add_argument(
        "--start_timesteps", default=1e4, type=int
    )  # How many time steps purely random policy is run for
    parser.add_argument(
        "--eval_freq", default=5e3, type=float
    )  # How often (time steps) we evaluate
    parser.add_argument(
        "--max_timesteps", default=1e6, type=float
    )  # Max time steps to run environment for
    parser.add_argument(
        "--save_models", action="store_true"
    )  # Whether or not models are saved
    parser.add_argument(
        "--expl_noise", default=0.1, type=float
    )  # Std of Gaussian exploration noise
    parser.add_argument(
        "--batch_size", default=100, type=int
    )  # Batch size for both actor and critic
    parser.add_argument("--discount", default=0.99, type=float)  # Discount factor
    parser.add_argument(
        "--tau", default=0.005, type=float
    )  # Target network update rate
    parser.add_argument(
        "--policy_noise", default=0.2, type=float
    )  # Noise added to target policy during critic update
    parser.add_argument(
        "--noise_clip", default=0.5, type=float
    )  # Range to clip target policy noise
    parser.add_argument(
        "--policy_freq", default=2, type=int
    )  # Frequency of delayed policy updates
    parser.add_argument(
        "--check_collision", action="store_true"
    )  # whether to check for collision or not
    parser.add_argument(
        "--stop_on_hit", action="store_true"
    )  # if check collision is true, then whether to end episode on collision
    parser.add_argument(
        "--collision_reward", default=-10, type=int
    )  # if check_collision is true, then what reward should be added to current reward
    parser.add_argument(
        "--valid_goals", action="store_true"
    )  # while training, whether to sample valid goals or not
    parser.add_argument(
        "--reaching_reward", default=1, type=int
    )  # reward to provide on successfully reaching the goal
    parser.add_argument(
        "--action_reward_coeff", default=0, type=float
    )  # what coeff factor to apply on action norm while calculating the reward

    args = parser.parse_args()

    file_name = "{}_{}".format(args.env_name, str(args.seed))

    root_path = "/".join(i for i in os.path.abspath(__file__).split("/")[:-1])
    sys.path.insert(0, root_path)
    import envs

    base_path = os.path.join(root_path, "log", args.exp_name)
    if not os.path.exists(os.path.join(base_path, "results")):
        os.makedirs(os.path.join(base_path, "results"))
    if args.save_models and not os.path.exists(
        os.path.join(base_path, "pytorch_models")
    ):
        os.makedirs(os.path.join(base_path, "pytorch_models"))

    env = gym.make(args.env_name)
    env._reaching_rew = args.reaching_reward
    env._action_rew_coeff = args.action_reward_coeff
    if args.check_collision:
        env._collision_check = args.check_collision
        env._collision_reward = args.collision_reward
        if args.stop_on_hit:
            env._stop_on_hit = True
    if args.valid_goals:
        env._valid_goal = True

    # Set seeds
    env.seed(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    max_action = float(env.action_space.high[0])

    # Initialize policy
    policy = TD3.TD3(state_dim, action_dim, max_action)

    replay_buffer = utils.ReplayBuffer()

    # Evaluate untrained policy
    evaluations = [evaluate_policy(policy)]

    total_timesteps = 0
    timesteps_since_eval = 0
    episode_num = 0
    done = True

    while total_timesteps < args.max_timesteps:
        if done:
            if total_timesteps != 0:
                print(
                    "Total T: {} Episode Num: {} Episode T: {} Reward: {}".format(
                        total_timesteps, episode_num, episode_timesteps, episode_reward
                    )
                )
                policy.train(
                    replay_buffer,
                    episode_timesteps,
                    args.batch_size,
                    args.discount,
                    args.tau,
                    args.policy_noise,
                    args.noise_clip,
                    args.policy_freq,
                )

            # Evaluate episode
            if timesteps_since_eval >= args.eval_freq:
                timesteps_since_eval %= args.eval_freq
                evaluations.append(evaluate_policy(policy))

                if args.save_models and max(evaluations) == evaluations[-1]:
                    policy.save(file_name, directory=base_path + "/pytorch_models")
                np.save(os.path.join(base_path, "results", file_name), evaluations)

            # Reset environment
            obs = env.reset()
            done = False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1

        # Select action randomly or according to policy
        if total_timesteps < args.start_timesteps:
            action = env.action_space.sample()
        else:
            action = policy.select_action(np.array(obs))
            if args.expl_noise != 0:
                action = (
                    action
                    + np.random.normal(
                        0, args.expl_noise, size=env.action_space.shape[0]
                    )
                ).clip(env.action_space.low, env.action_space.high)

        # Perform action
        new_obs, reward, done, _ = env.step(action)
        done_bool = (
            0 if episode_timesteps + 1 == env._max_episode_steps else float(done)
        )
        episode_reward += reward

        # Store data in replay buffer
        replay_buffer.add((obs, new_obs, action, reward, done_bool))

        obs = new_obs

        episode_timesteps += 1
        total_timesteps += 1
        timesteps_since_eval += 1

    # Final evaluation
    evaluations.append(evaluate_policy(policy))
    if args.save_models and max(evaluations) == evaluations[-1]:
        policy.save(file_name, directory=base_path + "/pytorch_models")
    np.save(os.path.join(base_path, "results", file_name), evaluations)
