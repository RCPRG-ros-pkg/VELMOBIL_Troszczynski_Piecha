#!/usr/bin/env python3

import rclpy
import numpy as np
from gymnasium import Env
from gymnasium.utils import seeding
from gymnasium.spaces import Dict, Box

from rl_tools.ignition_connection import IgnitionConnection


class IgnitionEnvironment(IgnitionConnection, Env):
    def __init__(self):
        super().__init__()
        
        self.get_logger().info("Ignition connections have been initialized, e.g., topics/services")
        
        self.episode_num = 0
        self.cumulated_episode_reward = 0
        ## idk może jakieś parsowanie z yaml czy coś by się tutaj przydało na przyszłosć? Potrzebujemy wymyślić zgrabny sposób na parametryzację środowiska.
        ## Też powstanie SimulationManager, który będzie zarządzał symulacją, obiektami które się w niej znajdują itp itd.

        self.max_linear_vel = 1.0
        self.min_linear_vel = 0.0
        self.max_angular_vel = 0.5
        self.min_angular_vel = -0.5

        self.min_obstacle_dist = 0.2
        
        self.goal_coordinates = np.array([5.0, 5.0], dtype=np.float32)
        self.goal_success_distance = 0.5

        self.action_space = Box(low=np.array([self.min_linear_vel, self.min_linear_vel, self.min_angular_vel]), 
                                high=np.array([self.max_linear_vel, self.max_linear_vel, self.max_angular_vel]),
                                dtype=np.float32)
        
        obs_shape = 360 + 6 + 1 + 1 # 360 laser readings + 6 odometry readings (x, y, theta, vx, vy, vtheta) + goal distance + headning
        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(obs_shape,), dtype=np.float32)

        self.last_action = np.zeros(3) # replay buffer czy coś...
   
    def spin(self):
        self.done_laser_ = False
        self.done_odom_ = False
        while not self.done_laser_ and not self.done_odom_:
            rclpy.spin_once(self) 
    
    def step(self, action):
        self._set_action(action)
        self.spin()
        
        obs = self._get_obs()
        done = self._is_done()
        info = {}
        
        reward = self._compute_reward(obs, done, action)
        self.cumulated_episode_reward += reward
        
        self.last_action = action
        
        return obs, reward, done, False, info
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        self.get_logger().info("Resetting gazebo environment")      
        
        self.reset_sim()
        self._update_episode()
        
        self.spin()
        obs = self._get_obs()
        
        self.get_logger().info("End resetting gazebo environment")
        
        self.last_action = np.zeros(3)
        
        info = {}
        
        return obs, info
    
    def render(self):
        pass
    
    def close(self):
        self.destroy_node()
     
    def _update_episode(self):
        self.get_logger().info("Reward = "+str(self.cumulated_episode_reward)+", EP = "+str(self.episode_num))
        
        self.episode_num += 1
        self.cumulated_episode_reward = 0

    def _set_action(self, action):
        action_linear_x = ((self.max_linear_vel * (action[0] + 1)) +
                         (self.min_linear_vel * (1 - action[0]))) / 2
        
        action_linear_y = ((self.max_linear_vel * (action[1] + 1)) +
                    (self.min_linear_vel * (1 - action[1]))) / 2
        action_angular = ((self.max_angular_vel * (action[2] + 1)) +
                          (-self.min_angular_vel * (1 - action[2]))) / 2
        
        comb_action = np.array([action_linear_x, action_linear_y, action_angular], dtype=np.float32)
        self.publish_velocity(comb_action)
         
    def _get_obs(self):
        observations = self.get_laser_data()
        odometry = self.get_odometry_data()

        obs = np.concatenate([observations, odometry])
        
        return obs   
    
    def _is_done(self):
        distances = self.get_laser_data()
        done = any(distances < self.min_obstacle_dist)
        if done:
            self.get_logger().info("Robot Crashed...") 
        
        return done
    
    def _compute_reward(self, obs, done, action):
        linear_velocity = self.get_current_linear_vel()
        progress_reward = linear_velocity * 5

        min_distance = np.min(obs)
        safety_penalty = -1.0 * (self.min_obstacle_dist - min_distance) if min_distance < self.min_obstacle_dist else 0.0

        crash_penalty = -20.0 if done else 0.0

        alive_reward = 0.1
 
        # to poniżej brzmi jak strategia do nagradzania oscylacji 
        if np.array_equal(action, self.last_action):
            repeat_action_penalty = -2.0
        else:
            repeat_action_penalty = 0.0

        reward = progress_reward + safety_penalty + crash_penalty + alive_reward + repeat_action_penalty
        
        return reward