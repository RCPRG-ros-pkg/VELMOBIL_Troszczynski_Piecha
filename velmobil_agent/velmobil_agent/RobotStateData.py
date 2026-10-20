#!/usr/bin/env python3
from __future__ import annotations
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from velmobil_agent.utilities.preprocessing import MinPooling, IPAPRecN


"""
Robot States Representations (LiDAR + Odometry)
+ Every state information source is encapsulated and preprocessed by RobotStateData unit
+ wait for robot state input -> preprocess -> 'acquire' -> let know RobotStateDataManager -> It encapsulates acquired data and forwards to DRL model
"""

# STEP mode is activated if timestamp_difference is passed

class RobotStateData:
    def __init__(self, global_data_acquisitor : RobotStateDataManager, timestamp_difference : float = None):
        self.robot_state_data_ = {}
        self.open_for_data_ = False
        self.last_acquired_timestamp = 0.0
        self.timestamp_difference = timestamp_difference
        self.acquired_ = False
        self.global_data_acquisitor = global_data_acquisitor 
        self.acquisition_lock = threading.Lock()
    def release(self):
        with self.acquisition_lock:
            self.robot_state_data_ = {}
            self.acquired_ = False
            self.last_acquired_timestamp = 0.0
    def get_acquired(self):
        return self.acquired_
    def get_robot_state_data(self):
        return self.robot_state_data_
    
    def acquire_data(self, data):
        with self.acquisition_lock:
            if self.acquired_ or not self.open_for_data_:
                return
            
            if self.timestamp_difference is not None:
                current_timestamp = data.header.stamp.sec + data.header.stamp.nanosec * 1.0e-9
                if not current_timestamp - self.last_acquired_timestamp > self.timestamp_difference:
                    return
                self.last_acquired_timestamp = current_timestamp

            self.acquired_ = True
            self.robot_state_data_ : dict = self.convert_acquired_data(data)
        self.global_data_acquisitor.acquired()

                




    def convert_acquired_data(self, data):
        return {}


class LidarStateData(RobotStateData):
    def __init__(self, global_data_acquisitor : RobotStateDataManager, timestamp_difference : float = None):
        super().__init__(global_data_acquisitor, timestamp_difference)
        self.range_max = 8.0
        self.range_min = 0.06
        self.min_pool = MinPooling(360, 90)
        self.ipaprecn = IPAPRecN(beta=0.0)

    def convert_acquired_data(self, data: LaserScan):
        lidar_data = {}
        lidar_data["raw_ranges"] = np.nan_to_num(np.array(data.ranges), nan=self.range_max, posinf=self.range_max, neginf=self.range_min)
        ranges_clipped = np.clip(lidar_data["raw_ranges"], self.range_min, self.range_max)
        lidar_data["lin_norm_ranges"] = (ranges_clipped - self.range_min) / (self.range_max - self.range_min)
        lidar_data["processed_ranges"] = self.ipaprecn.process(self.min_pool.process(ranges_clipped)) # IPAPRecN + MinPooling
        return lidar_data



class OdomStateData(RobotStateData):
    def __init__(self, global_data_acquisitor : RobotStateDataManager, timestamp_difference : float = None):
        super().__init__(global_data_acquisitor, timestamp_difference)
        self.collect_start_point = False

    def set_current_goal(self, goal: np.ndarray):
        self.current_goal = goal
        self.collect_start_point = True
    
    def convert_acquired_data(self, data: Odometry):
        odom_data = {}
        quaternion = data.pose.pose.orientation
        yaw = np.arctan2(2.0*(quaternion.w*quaternion.z + quaternion.x*quaternion.y), 1.0 - 2.0*(quaternion.y*quaternion.y + quaternion.z*quaternion.z))
        odom_data["current_pose"] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, yaw])
        odom_data["current_velocity"] = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z])
        odom_data["current_distance"] = np.linalg.norm(self.current_goal[:2] - odom_data["current_pose"][:2])
        if self.collect_start_point:
            self.max_distance = odom_data["current_distance"]
            self.collect_start_point = False
        relative_goal_pose = np.array([self.current_goal[0] - odom_data["current_pose"][0], self.current_goal[1] - odom_data["current_pose"][1]])
        bearing_raw = np.arctan2(relative_goal_pose[1], relative_goal_pose[0]) - yaw
        odom_data["bearing"] = np.arctan2(np.sin(bearing_raw), np.cos(bearing_raw)) # Right now bearing is linearly normalized. Maybe it should be nonlinear...
        odom_data["normalized_distance"] = np.concatenate([np.array([odom_data["current_distance"] / self.max_distance, odom_data["bearing"] / np.pi]), odom_data["current_velocity"]])
        return odom_data



"""
Robot State Data Manager
+ Waits for data from all RobotStateData units
+ When all data is acquired, it pushes data further into pipeline (training or inference)
"""

## Brakuje jeszcze przekazywania Goal do OdomStateData, oraz jakiegoś mechanizmu informowania czy wgl zbierać dane czy nie (bo np. epizod nie ruszył)
class RobotStateDataManager:
    def __init__(self, agent_node : Node, lidar_timestamp_difference : float = None, odom_timestamp_difference : float = None):
        self.agent_node = agent_node
        self.state_data_list : list[RobotStateData] = []
        self.state_data_lock = threading.Lock()
        self.initialize_state_data(lidar_timestamp_difference, odom_timestamp_difference)

    def open_for_data(self):
        with self.state_data_lock:
            for sd in self.state_data_list:
                sd.open_for_data_ = True
                sd.release()

    def close_for_data(self):
        with self.state_data_lock:
            for sd in self.state_data_list:
                sd.open_for_data_ = False
                sd.release()
    
    def initialize_state_data(self, lidar_timestamp_difference, odom_timestamp_difference):
        self.lidar_state_data = LidarStateData(self, lidar_timestamp_difference)
        self.odom_state_data = OdomStateData(self, odom_timestamp_difference)
        self.state_data_list.append(self.lidar_state_data)
        self.state_data_list.append(self.odom_state_data)

    def release_all(self):
        with self.state_data_lock:
            for sd in self.state_data_list:
                sd.release()
    
    def acquired(self):
        with self.state_data_lock:
            if all(sd.get_acquired() for sd in self.state_data_list):
                concatenated_data : dict = {
                    "lidar" : self.lidar_state_data.get_robot_state_data(),
                    "odom" : self.odom_state_data.get_robot_state_data()
                }
                self.agent_node.predict_action(concatenated_data)



class RobotActionDataManager:
    def __init__(self, agent_node: Node):
        self.agent_node = agent_node

    def send_action_data(self, action_data):
        msg = Twist()
        # self.get_logger().info(f"{type(float(action_data[0]))}")
        msg.linear.x = float(action_data[0])
        msg.linear.y = float(action_data[1])
        msg.angular.z = float(action_data[2])
        self.agent_node.cmd_vel_pub.publish(msg) 