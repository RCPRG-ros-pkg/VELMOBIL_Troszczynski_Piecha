#!/usr/bin/env python3
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from .RobotStateData import RobotStateDataManager, RobotActionDataManager
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import gymnasium as gym
from gymnasium.spaces import Box
from ros_gz_interfaces.srv import ControlWorld

"""
AgentTrainingDRL
+ Odpowiednik AgentDRL, ale dla treningu: to SB3 (model.learn()) woła step(action),
  więc nie ma tu action servera ani wewnętrznego model.predict() - akcja przychodzi z zewnątrz.

"""

class AgentTrainingDRL(Node, gym.Env):
    def __init__(self, goal_sampler=None):
        Node.__init__(self, "agent_training_drl_node")
        gym.Env.__init__(self)

        self.goal_sampler = goal_sampler

        self.step_ready = threading.Event()

        # Potem trzeba to do jakiegoś YAML dać te parametry...
        self.max_steps_per_episode = 500
        self.collision_range = 0.5
        self.goal_tolerance = 0.2
        self.steps_this_episode = 0

        self.initialize_action_utils()
        self.initialize_state_utils()

    def initialize_action_utils(self):
        self.robot_action_data_manager = RobotActionDataManager(self)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.action_space = Box(low=np.array([-1.0, -1.0, -1.0]),
                                high=np.array([1.0, 1.0, 1.0]),
                                dtype=np.float32)

    def initialize_state_utils(self):
        self.robot_state_data_manager = RobotStateDataManager(self)
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_fusion", self.robot_state_data_manager.lidar_state_data.acquire_data, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.robot_state_data_manager.odom_state_data.acquire_data, qos_profile_sensor_data)

        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(360 + 5,), dtype=np.float32)

    def predict_action(self, data):
        self.get_logger().info("Predict action")
        self.latest_full_state = data
        self.latest_state = np.concatenate([data["lidar"]["normalized_ranges"], data["odom"]["normalized_distance"]]) # 360(norm-lidar) + 5(norm-distance,norm-bearing,vx,vy,omega)
        self.step_ready.set()

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_state_data_manager.close_for_data()
        self.get_logger().info(f"ENV RESE3")
        self.cmd_vel_pub.publish(Twist())

        self.reset_simulation()

        goal = self.goal_sampler() if self.goal_sampler is not None else np.array([3.0, 0.0, 0.0])
        self.robot_state_data_manager.odom_state_data.set_current_goal(goal)

        self.steps_this_episode = 0
        self.step_ready.clear()
        self.robot_state_data_manager.open_for_data()
        self.get_logger().info(f"ENV RESE4")

        if not self.step_ready.wait(timeout=5.0):
            raise RuntimeError("Brak danych stanu po resecie - sprawdź, czy /lidar_fusion i /odom publikują")
        return self.latest_state, {}

    def step(self, action):
        self.get_logger().info(f"Step: {self.steps_this_episode}")
        self.steps_this_episode += 1
        self.robot_action_data_manager.send_action_data(action) # wykonaj akcję

        self.step_ready.clear() # zwaolnij flagę eventu, czekając na nowe dane stanu
        self.robot_state_data_manager.release_all() # uwolnij zatrzaśnięte dane, żeby nowe mogły przyjść
        self.get_logger().info(f"After release all")

        if not self.step_ready.wait(timeout=3.0): # wciągu trzech sekund powinna się wywołać metoda predict_action() [tutaj nic nie predictuje - tylko ustawia flagę eventu o nowych danych]
            return self.latest_state, 0.0, False, True, {"timeout": True}
        

        observation = self.latest_state
        reward, terminated, truncated, info = self.compute_step_result()
        return observation, reward, terminated, truncated, info

    def compute_step_result(self):

        STEP_WEIGHT = 0.01
        DISTANCE_WEIGHT = 1.0
        BEARING_WEIGHT = 0.5

        raw_lidar = self.latest_full_state["lidar"]["raw_ranges"]
        goal_distance = self.latest_full_state["odom"]["current_distance"]
        collided = np.any(raw_lidar < self.collision_range)
        reached_goal = goal_distance < self.goal_tolerance
        terminated = collided or reached_goal
        truncated = self.steps_this_episode >= self.max_steps_per_episode

        reward = 0.0
        if reached_goal:
            reward = 10.0
        elif not collided:
            constant_step_reward = -STEP_WEIGHT
            distance_reward = DISTANCE_WEIGHT * (1 - self.latest_full_state["odom"]["normalized_distance"][0]) ** 3
            bearing_reward = -BEARING_WEIGHT * self.latest_full_state["odom"]["normalized_distance"][1]
            reward = constant_step_reward + distance_reward + bearing_reward
        else:
            reward = -50.0

        info = {"collided": collided, "reached_goal": reached_goal}
        return reward, terminated, truncated, info



    def reset_simulation(self):
        pass
