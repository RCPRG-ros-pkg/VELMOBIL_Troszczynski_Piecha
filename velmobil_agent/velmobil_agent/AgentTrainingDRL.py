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
from velmobil_interfaces.srv import StageModification
from std_srvs.srv import Trigger

from velmobil_agent.utilities.reward_function import TrainingSignals, RewardFunction

"""
Agent Training DRL
+ Wraps Ignition+ROS with gymnasium.Env interface
+ Gets Sensor Data from RobotActionDataManager
    - Implements reset(), step()
+ Implements Reward function
"""
WORLD_CONTROL_MULTI_STEP = 11
COLLISION_RANGE = 0.8

# max_step_size = 0.01 => one sim step = 0.01s
# LiDAR Rate : 20 Hz -> 0.05s 
# Odom timestamp_difference = 0.085

# LIDAR_TIMESTAMP_DIFFERENCE = 0.055
# ODOM_TIMESTAMP_DIFFERENCE = 0.095
LIDAR_TIMESTAMP_DIFFERENCE = None
ODOM_TIMESTAMP_DIFFERENCE = None


class AgentTrainingDRL(Node, gym.Env):
    def __init__(self):
        Node.__init__(self, "agent_training_drl_node", automatically_declare_parameters_from_overrides=True)
        gym.Env.__init__(self)

        self.episode_reward = 0.0
        self.steps_this_episode = 0
        self.total_episodes = 0
        self.episode_rewards = []

        # TEMP!!
        self.start_position = np.array([0.0, 0.0])
        self.goal_position = np.array([5.0, 0.0])
        self.goal_tolerance = 1.0
        self.previous_action_vector = np.array([0.0, 0.0, 0.0])

        self.new_data_event = threading.Event()
        self.stepped_event = threading.Event()
        self.reset_simulation_event = threading.Event()


        self.max_steps_per_episode = self.get_parameter('max_steps_per_episode').value
        self.stage_shift_episodes : list[int] = self.get_parameter('stage_shift_episodes').value
        self.quick_reroll_episode_rate = self.get_parameter('quick_reroll_episode_rate').value
        self.hard_reroll_episode_rate = self.get_parameter('hard_reroll_episode_rate').value

        self.initialize_action_utils()
        self.initialize_state_utils()

        self.simulation_step_client = self.create_client(ControlWorld, '/world/velmobil_arena/control')
        self.quick_reset_simulation_client = self.create_client(Trigger, '/quick_reset_simulation')
        self.hard_reset_simulation_client = self.create_client(StageModification, '/hard_reset_simulation')



    def initialize_action_utils(self):
        self.robot_action_data_manager = RobotActionDataManager(self)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.action_space = Box(low=np.array([-1.0, -1.0, -1.0]),
                                high=np.array([1.0, 1.0, 1.0]),
                                dtype=np.float32)

    def initialize_state_utils(self):
        self.robot_state_data_manager = RobotStateDataManager(self, lidar_timestamp_difference=LIDAR_TIMESTAMP_DIFFERENCE, odom_timestamp_difference=ODOM_TIMESTAMP_DIFFERENCE)
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_fusion", self.robot_state_data_manager.lidar_state_data.acquire_data, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, "/model/velmobil/odometry_with_covariance", self.robot_state_data_manager.odom_state_data.acquire_data, 10)
        self.robot_state_data_manager.odom_state_data.set_current_goal(self.goal_position)   ### TEMP!!!

        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(90 + 5,), dtype=np.float32)

    def predict_action(self, data):
        self.latest_full_state = data
        self.latest_state = np.concatenate([data["lidar"]["processed_ranges"], data["odom"]["normalized_distance"]]) # 90(processed-lidar) + 5(norm-distance,norm-bearing,vx,vy,omega)
        self.new_data_event.set()

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.steps_this_episode = 0
        self.robot_state_data_manager.close_for_data()
        self.cmd_vel_pub.publish(Twist())

        self.stepped_event.clear()
        self.new_data_event.clear()

        self.reset_simulation_event.clear()
        if self.total_episodes == self.stage_shift_episodes[0]:
            self.stage_shift_episodes.append(self.stage_shift_episodes.pop(0))
            self.get_logger().info("[Reset] Stage Up")
            self.hard_reset_simulation(stage_up=True)
        elif (self.total_episodes % self.hard_reroll_episode_rate) == 0:
            self.get_logger().info("[Reset] Stage Reroll")
            self.hard_reset_simulation(stage_reroll=True)
        elif (self.total_episodes % self.quick_reroll_episode_rate) == 0:
            self.get_logger().info("[Reset] New Start & Goal")
            self.hard_reset_simulation(start_goal_reroll=True)
        else:    
            self.quick_reset_simulation()

        if not self.reset_simulation_event.wait(timeout=10.0):
            self.get_logger().warn("[Reset] Timeout waiting for simulation step!")

        self.total_episodes += 1

        self.robot_state_data_manager.zero_last_acquired_timestamp_all()
        self.robot_state_data_manager.release_all() 
        self.robot_state_data_manager.open_for_data()

        self.step_simulation()


        if not self.stepped_event.wait(timeout=6.0):
            self.get_logger().warn("[Reset] Timeout waiting for simulation step!")
        if not self.new_data_event.wait(timeout=6.0):
            self.get_logger().warn("[Reset] Timeout waiting for new state data!")   
                
        observation = self.latest_state
        self.previous_normalized_goal_distance = self.latest_full_state["odom"]["normalized_distance"][0]
        return observation, {}



    def step(self, action):
        # 1. Publish action
        self.steps_this_episode += 1
        self.current_action_vector = np.asarray(action, dtype=np.float64)
        self.robot_action_data_manager.send_action_data(action)

        # 2. Release data manager, so it can wait for new state data
        self.new_data_event.clear()
        self.robot_state_data_manager.release_all()

        # 3. Step simulation and pause it
        self.stepped_event.clear()
        self.step_simulation()
        if not self.stepped_event.wait(timeout=6.0):
            self.get_logger().warn("[Step] Timeout waiting for simulation step!")
            observation = self.latest_state
            return observation, 0.0, False, True, {"timeout": True}

        # 4. Wait for data collected after stepping simulation
        if not self.new_data_event.wait(timeout=6.0):
            self.get_logger().warn("[Step] Timeout waiting for new state data!")
            observation = self.latest_state
            return observation, 0.0, False, True, {"timeout": True}
        
        observation = self.latest_state
        reward, terminated, truncated, info = self.compute_step_result()

        return observation, reward, terminated, truncated, info

    def quick_reset_simulation(self):
        self.robot_state_data_manager.odom_state_data.set_current_goal(self.goal_position)
        self.previous_action_vector = np.array([0.0, 0.0, 0.0])
        self.episode_reward = 0.0
        future = self.quick_reset_simulation_client.call_async(Trigger.Request())
        future.add_done_callback(self.quick_reset_simulation_callback)

    def quick_reset_simulation_callback(self, future):
        result : Trigger.Response = future.result()
        self.reset_simulation_success = result is not None and result.success
        if result.success:
            self.get_logger().info(f"{result.message}")
            self.reset_simulation_event.set()
        else:
            self.get_logger().error(f"Couldn't reset the simulation: {result.message if result else 'no response'}")

    def hard_reset_simulation(self, stage_reroll: bool = False, start_goal_reroll: bool = False, stage_up: bool = False):
        self.previous_normalized_goal_distance = 1.0
        self.episode_reward = 0.0
        req = StageModification.Request()
        if start_goal_reroll:
            req.change_start_goal = True
        elif stage_reroll:
            req.change_stage = StageModification.Request.STAGE_REROLL
        elif stage_up:
            req.change_stage = StageModification.Request.STAGE_UP
        future = self.hard_reset_simulation_client.call_async(req)
        future.add_done_callback(self.hard_reset_simulation_callback)

    def hard_reset_simulation_callback(self, future):
        result : StageModification.Response = future.result()
        if result.success:
            self.get_logger().info(f"{result.message}")
            self.start_position = [result.initial_pos_x, result.initial_pos_y]
            self.goal_position = [result.goal_pos_x, result.goal_pos_y]
            self.goal_tolerance = result.goal_tolerance
            self.robot_state_data_manager.odom_state_data.set_current_goal(self.goal_position)
        self.reset_simulation_event.set()


    def step_simulation(self):
        req = ControlWorld.Request()
        req.world_control.pause = True
        # req.world_control.step = True
        req.world_control.multi_step = WORLD_CONTROL_MULTI_STEP
        future = self.simulation_step_client.call_async(req)
        future.add_done_callback(self.step_simulation_callback)

    def step_simulation_callback(self, future):
        result = future.result()
        if result.success:
            self.stepped_event.set()
        else:
            self.get_logger().error("Failed to step simulation!")
        

    def compute_step_result(self):
        training_signals = TrainingSignals(
            self.latest_full_state["lidar"]["raw_ranges"],
            self.latest_full_state["odom"]["current_distance"],
            self.latest_full_state["odom"]["normalized_distance"][0],
            self.latest_full_state["odom"]["normalized_distance"][1],
            self.latest_full_state["odom"]["current_velocity"][2],
            self.previous_normalized_goal_distance,
            self.current_action_vector,
            self.previous_action_vector
        )

        reward_function = RewardFunction('rf_3')
        reward, collided, reached_goal = reward_function(training_signals, COLLISION_RANGE, self.goal_tolerance)

        terminated = collided or reached_goal
        truncated = self.steps_this_episode >= self.max_steps_per_episode

        self.episode_reward += reward
        self.previous_normalized_goal_distance = training_signals.normalized_goal_distance
        self.previous_action_vector = training_signals.current_action_vector
        if terminated or truncated:
            self.episode_rewards.append(self.episode_reward)
            self.get_logger().info(f"Episode {self.total_episodes} ended. Steps: {self.steps_this_episode}, Collided: {collided}, Reached Goal: {reached_goal}, Distance to Goal: {training_signals.goal_distance:.2f}, Episode Reward: {self.episode_reward:.2f}")

        info = {"collided": collided, "reached_goal": reached_goal}
        return reward, terminated, truncated, info
