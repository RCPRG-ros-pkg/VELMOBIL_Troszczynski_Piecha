#!/usr/bin/env python3
import numpy as np
import sys
sys.modules['cv2'] = None 
import flax.linen as nn
from velmobil_agent.common.model_configuration import ModelConfig
import sbx


"""
Model
+ Wraps an SB3/SBX policy so AgentDRL don't touch SB3/SBX directly.
+ Use ModelConfig abstraction class to initialize hiperparameters and DNN architectures
+ Two initialization paths:
    - model_path given -> load a trained policy for inference (env optional, only
      needed if you intend to keep training the loaded policy).
    - env given, no model_path -> fresh policy.
"""

class Model:
    def __init__(self, env=None, model_path: str = None, algorithm_name: str = "PPO", algorithm_configuration: str = 3, total_timesteps: int = 250000, **kwargs):
        self.algorithm_name = algorithm_name
        self.algorithm_configuration = algorithm_configuration
        algorithm = getattr(sbx, algorithm_name)
        if model_path is not None:
            self.algo = algorithm.load(model_path, env=env)
        elif env is not None:
            self.total_timesteps, hiperparams, policy_kwargs, actor_critic = self.load_config(total_timesteps)
            self.algo = algorithm("MlpPolicy", env, **hiperparams, policy_kwargs = policy_kwargs, **kwargs) #   "SimbaPolicy" - https://arxiv.org/pdf/2410.09754v2
        else:
            raise ValueError("Model needs either model_path (inference) or env (fresh training)")

    def load_config(self, total_timesteps) -> tuple[int, dict, dict, nn.Module | None]:
        model_configuration = ModelConfig.choose_model_config(f"{self.algorithm_name}Config{self.algorithm_configuration}", total_timesteps)
        return model_configuration()

    def predict(self, observation, deterministic: bool = True):
        action, _ = self.algo.predict(observation, deterministic=deterministic)
        return action

    def train(self, **kwargs):
        self.algo.learn(total_timesteps=self.total_timesteps, **kwargs)

    def save(self, path: str):
        self.algo.save(path)
