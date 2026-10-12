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
from velmobil_interfaces.srv import TrainingPhase
from std_srvs.srv import Trigger

"""
Agent Training DRL
+ Wraps Ignition+ROS with gymnasium.Env interface
+ Gets Sensor Data from RobotActionDataManager
    - Implements reset(), step()
+ Implements Reward function
"""

# LiDAR Rate : 20 Hz -> 0.05s -> 50ms => LiDAR timestamp difference = 0.04
# Odom timestamp_difference = 0.085

LIDAR_TIMESTAMP_DIFFERENCE = 0.145
ODOM_TIMESTAMP_DIFFERENCE = 0.195
WORLD_CONTROL_MULTI_STEP = 200


class AgentTrainingDRL(Node, gym.Env):
    def __init__(self):
        Node.__init__(self, "agent_training_drl_node")
        gym.Env.__init__(self)

        self.episode_reward = 0.0


        # TEMP!!
        self.initial_position = np.array([0.0, 0.0])
        self.goal_position = np.array([5.0, 0.0])
        self.goal_tolerance = 1.0
        self.previous_normalized_goal_distance = 1.0

        self.new_data_event = threading.Event()
        self.stepped_event = threading.Event()
        self.reset_simulation_event = threading.Event()

        self.declare_parameter('max_steps_per_episode', 200)
        self.declare_parameter('collision_range', 1.0)

        self.max_steps_per_episode = self.get_parameter('max_steps_per_episode').value
        self.collision_range = self.get_parameter('collision_range').value
        self.steps_this_episode = 0

        self.initialize_action_utils()
        self.initialize_state_utils()

        # self.simulation_manager_client = self.create_client(TrainingPhase, '/training_phase')
        self.simulation_step_client = self.create_client(ControlWorld, '/world/velmobil_arena/control')
        self.reset_simulation_client = self.create_client(Trigger, '/reset_simulation')


    def initialize_action_utils(self):
        self.robot_action_data_manager = RobotActionDataManager(self)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.action_space = Box(low=np.array([-1.0, -1.0, -1.0]),
                                high=np.array([1.0, 1.0, 1.0]),
                                dtype=np.float32)

    def initialize_state_utils(self):
        self.robot_state_data_manager = RobotStateDataManager(self, lidar_timestamp_difference=LIDAR_TIMESTAMP_DIFFERENCE, odom_timestamp_difference=ODOM_TIMESTAMP_DIFFERENCE)
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_fusion", self.robot_state_data_manager.lidar_state_data.acquire_data, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, "/model/velmobil/odometry_with_covariance", self.robot_state_data_manager.odom_state_data.acquire_data, qos_profile_sensor_data)
        self.robot_state_data_manager.odom_state_data.set_current_goal(self.goal_position)   ### TEMP!!!

        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(360 + 5,), dtype=np.float32)

    def predict_action(self, data):
        self.latest_full_state = data
        self.latest_state = np.concatenate([data["lidar"]["normalized_ranges"], data["odom"]["normalized_distance"]]) # 360(norm-lidar) + 5(norm-distance,norm-bearing,vx,vy,omega)
        self.new_data_event.set()



    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_state_data_manager.close_for_data()
        self.cmd_vel_pub.publish(Twist())

        self.reset_simulation_event.clear()
        self.reset_simulation()
        if not self.reset_simulation_event.wait(timeout=3.0):
            self.get_logger().warn("Timeout waiting for simulation step!")

        self.steps_this_episode = 0

        self.new_data_event.clear()
        self.robot_state_data_manager.release_all() 
        self.robot_state_data_manager.open_for_data()
        if not self.new_data_event.wait(timeout=3.0):
            self.get_logger().warn("Timeout waiting for new state data!")
        
        observation = self.latest_state
        return observation, {}



    def step(self, action):
        # 1. Publish action
        self.steps_this_episode += 1
        self.robot_action_data_manager.send_action_data(action)

        # 2. Release data manager, so it can wait for new state data
        self.new_data_event.clear()
        self.robot_state_data_manager.release_all()

        # 3. Step simulation and pause it
        self.stepped_event.clear()
        self.step_simulation()
        if not self.stepped_event.wait(timeout=3.0):
            self.get_logger().warn("Timeout waiting for simulation step!")
            observation = self.latest_state
            return observation, 0.0, False, True, {"timeout": True}

        # 4. Wait for data collected after stepping simulation
        if not self.new_data_event.wait(timeout=3.0):
            self.get_logger().warn("Timeout waiting for new state data!")
            observation = self.latest_state
            return observation, 0.0, False, True, {"timeout": True}
        
        observation = self.latest_state
        reward, terminated, truncated, info = self.compute_step_result()

        return observation, reward, terminated, truncated, info

    def reset_simulation(self):
        self.robot_state_data_manager.odom_state_data.set_current_goal(self.goal_position)   ### TEMP!!!
        self.previous_normalized_goal_distance = 1.0
        self.episode_reward = 0.0
        future = self.reset_simulation_client.call_async(Trigger.Request())
        future.add_done_callback(self.reset_simulation_callback)

    def reset_simulation_callback(self, future):
        result : Trigger.Response = future.result()
        if not result.success:
            self.get_logger().error("Couldn't reset the simulation")
        else:
            self.get_logger().info(f"{result.message}")
            self.reset_simulation_event.set()

    def step_simulation(self):
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.step = True
        req.world_control.multi_step = WORLD_CONTROL_MULTI_STEP
        future = self.simulation_step_client.call_async(req)
        future.add_done_callback(self.step_simulation_callback)

    def step_simulation_callback(self, future):
        result = future.result()
        if not result.success:
            self.get_logger().error("Failed to step simulation!")
        else:
            self.stepped_event.set()
        






    def compute_step_result(self):

        REACH_GOAL_WEIGHT = 15.0
        CRASH_WEIGHT = -10.0
        STEP_WEIGHT = 0.005
        DISTANCE_WEIGHT = 10.0
        BEARING_WEIGHT = 1.0
        PROGRESS_WEIGHT = 20.0

        raw_lidar = self.latest_full_state["lidar"]["raw_ranges"]
        goal_distance = self.latest_full_state["odom"]["current_distance"]
        
        collided = bool(np.any(raw_lidar < self.collision_range))
        reached_goal = bool(goal_distance < self.goal_tolerance)
        terminated = collided or reached_goal
        truncated = self.steps_this_episode >= self.max_steps_per_episode

        reward = 0.0
        if reached_goal:
            reward = REACH_GOAL_WEIGHT
        elif collided:
            reward = CRASH_WEIGHT
        else:
            constant_step_reward = -STEP_WEIGHT
            # distance_reward = DISTANCE_WEIGHT * (1 - self.latest_full_state["odom"]["normalized_distance"][0]) ** 3
            # bearing_reward = -BEARING_WEIGHT * abs(self.latest_full_state["odom"]["normalized_distance"][1])
            progress_reward = PROGRESS_WEIGHT * (self.previous_normalized_goal_distance - self.latest_full_state["odom"]["normalized_distance"][0])
            reward = constant_step_reward + progress_reward # + distance_reward + bearing_reward

        self.episode_reward += reward
        self.previous_normalized_goal_distance = self.latest_full_state["odom"]["normalized_distance"][0]
        if terminated or truncated:
            self.get_logger().info(f"Episode ended. Steps: {self.steps_this_episode}, Collided: {collided}, Reached Goal: {reached_goal}, Distance to Goal: {goal_distance:.2f}, Episode Reward: {self.episode_reward:.2f}")

        info = {"collided": collided, "reached_goal": reached_goal}
        return reward, terminated, truncated, info


