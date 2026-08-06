#!/usr/bin/env python3
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from .RobotStateData import RobotStateDataManager
from .Model import Model
from gymnasium import Env
from gymnasium.utils import seeding
from gymnasium.spaces import Dict, Box


"""AgentDRL - rosowy wrapper na model z SB3"""
class AgentDRL(Node, Env):
    def __init__(self, model: Model = None, mode: str = "inference"):
        Node.__init__(self, "agent_drl_node")
        Env.__init__(self)
        assert mode in ("inference", "training")
        self.mode = mode
        self.model = model

        self.initialize_action_utils()
        self.initialize_state_utils()

        self.latest_state = None
        self.state_ready_event = threading.Event()

    def initialize_action_utils(self):

        # tutaj jeszcze coś od RobotActionManager albo coś prostszego (potem dodam rzeczy rosowe typowo jak pub,sub,action)
        self.action_space = Box(low=np.array([-1.0, -1.0, -1.0]), high=np.array([-1.0, -1.0, -1.0]), dtype=np.float32)

    def initialize_state_utils(self):
        self.robot_state_data_manager = RobotStateDataManager(self)
        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(360 + 5,), dtype=np.float32) # 360 laser readings + 5 odometry-based readings (goal_distance, goal_bearing, vx, vy, omega)

    def acquire_state_data(self, data):
        self.latest_state = data

        if self.mode == "inference":
            action = self.model.predict(data)
            self.send_action(action)
        else:
            self.state_ready_event.set()


    def step(self, action):
        self.state_ready_event.clear()
        self.send_action(action)
        self.robot_state_data_manager.release_all()
        self.state_ready_event.wait()  # blocks until acquire_state_data() fires again

        observation = self.latest_state
        reward, terminated, truncated, info = self.compute_step_result(observation)
        return observation, reward, terminated, truncated, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_state_data_manager.release_all()
        self.state_ready_event.clear()
        ## tutaj setowanie nowego goal, i wait na nowy state
        self.state_ready_event.wait()
        return self.latest_state, {}

    def compute_step_result(self, observation):
        ##### PLACEHOLDER!!!
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        return reward, terminated, truncated, info

    def send_action(self, action):
        
        #tutaj na pewno release trzeba wrzucić...
        pass
