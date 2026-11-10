#!/usr/bin/env python3
import numpy as np
import sys
sys.modules['cv2'] = None 
import flax.linen as nn
from velmobil_agent.common.model_configuration import ModelConfig

from sbx import SAC as ALGORITHM
CONFIGURATION = 1


"""
Model
+ Wraps an SB3/SBX policy so AgentDRL don't touch SB3/SBX directly.
+ Use ModelConfig abstraction class to initialize hiperparameters and DNN architectures
+ Two initialization paths:
    - model_path given -> load a trained policy for inference (env optional, only
      needed if you intend to keep training the loaded policy).
    - env given, no model_path -> fresh policy.
"""

ALGORITHM_NAME = ALGORITHM.__name__


class Model:
    def __init__(self, env=None, model_path: str = None, algorithm = ALGORITHM, **kwargs):
        if model_path is not None:
            self.algo = algorithm.load(model_path, env=env)
        elif env is not None:
            self.total_timesteps, hiperparams, policy_kwargs, actor_critic = self.load_config()
            #   "SimbaPolicy" - https://arxiv.org/pdf/2410.09754v2
            self.algo = algorithm("SimbaPolicy", env, **hiperparams, policy_kwargs = policy_kwargs, **kwargs)
        else:
            raise ValueError("Model needs either model_path (inference) or env (fresh training)")

    def load_config(self) -> tuple[int, dict, dict, nn.Module | None]:
        model_configuration = ModelConfig.choose_model_config(f"{ALGORITHM_NAME}Config{CONFIGURATION}")
        return model_configuration()

    def predict(self, observation, deterministic: bool = True):
        action, _ = self.algo.predict(observation, deterministic=deterministic)
        return action

    def train(self, **kwargs):
        self.algo.learn(total_timesteps=self.total_timesteps, **kwargs)

    def save(self, path: str):
        self.algo.save(path)
