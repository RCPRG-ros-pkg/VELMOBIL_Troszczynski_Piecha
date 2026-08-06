#!/usr/bin/env python3
import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise

"""
Model
+ Wraps an SB3 policy so AgentDRL/TrainingManager don't touch SB3 directly.
+ Two initialization paths:
    - model_path given -> load a trained policy for inference (env optional, only
      needed if you intend to keep training the loaded policy).
    - env given, no model_path -> fresh policy, meant to be trained via TrainingManager.
"""


class Model:
    def __init__(self, env=None, model_path: str = None, **td3_kwargs):
        if model_path is not None:
            self.algo = TD3.load(model_path, env=env)
        elif env is not None:
            n_actions = env.action_space.shape[-1]
            action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
            self.algo = TD3("MlpPolicy", env, action_noise=action_noise, **td3_kwargs)
        else:
            raise ValueError("Model needs either model_path (inference) or env (fresh training)")

    def predict(self, observation, deterministic: bool = True):
        action, _ = self.algo.predict(observation, deterministic=deterministic)
        return action

    def train(self, total_timesteps: int, **kwargs):
        self.algo.learn(total_timesteps=total_timesteps, **kwargs)

    def save(self, path: str):
        self.algo.save(path)
