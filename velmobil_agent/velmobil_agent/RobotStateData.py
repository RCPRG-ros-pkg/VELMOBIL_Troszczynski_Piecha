#!/usr/bin/env python3
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

"""
Robot States Representations
+ Every state information source is encapsulated and preprocessed by RobotStateData unit
+ wait for robot state input -> preprocess -> 'acquire' -> let know ros-gym adapter, so it will make 'packed state'
"""

class RobotStateDataManager:
    pass

class RobotActionDataManager:
    pass


class RobotStateData:
    def __init__(self, global_data_acquisitor : RobotStateDataManager = None):
        self.robot_state_data_ = None
        self.open_for_data_ = False
        self.acquired_ = False
        self.global_data_acquisitor = global_data_acquisitor 
        self.acquisition_lock = threading.Lock()
    def release(self):
        self.robot_state_data_ = None
        self.acquired_ = False
        self.acquisition_lock.release() # żeby czasem nie zrobił się jakiś dziwny deadlock...
    def get_acquired(self):
        return self.acquired_
    def get_robot_state_data(self):
        return self.robot_state_data_
    
    def acquire_data(self, data):
        self.acquisition_lock.acquire()
        if not self.acquired_ and self.open_for_data_:
            self.acquired_ = True
            self.robot_state_data_ = self.convert_acquired_data(data)
            self.global_data_acquisitor.acquired(self)

    def convert_acquired_data(self, data):
        pass


class LidarStateData(RobotStateData):
    """Normalizes the laser scan data to a range of [0, 1]"""
    def __init__(self, global_data_acquisitor : RobotStateDataManager = None):
        super().__init__(global_data_acquisitor)
        self.range_max = 8.0
        self.range_min = 0.06

    def convert_acquired_data(self, data: LaserScan):
        ranges_raw = np.nan_to_num(np.array(data.ranges), nan=self.range_max, posinf=self.range_max, neginf=self.range_min)
        ranges_clipped = np.clip(ranges_raw, self.range_min, self.range_max)
        ranges_normalized = (ranges_clipped - self.range_min) / (self.range_max - self.range_min)
        return ranges_normalized


# class ImuStateData(RobotStateData):
# Na ten moment sama odometria

class OdomStateData(RobotStateData):
    """Casts odometry data to [distance, bearing, vx, vy, omega]"""
    def __init__(self, global_data_acquisitor : RobotStateDataManager = None):
        super().__init__(global_data_acquisitor)
        self.collect_start_point = False
        self.max_distance = None
        self.current_distance = None
        self.current_goal = None
    def set_current_goal(self, goal: np.ndarray):
        self.current_goal = goal
        self.collect_start_point = True
    
    def convert_acquired_data(self, data: Odometry):
        current_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y, 2 * np.arcsin(data.pose.pose.orientation.z)]) #arcsin bo to jest z quaterniona, a chcemy yaw.
        current_velocity = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist])
        self.current_distance = np.linalg.norm(self.current_goal[:2] - current_pose[:2])
        if self.collect_start_point:
            self.max_distance = self.current_distance
        relative_goal_pose = np.array([self.current_goal[0] - current_pose[0], self.current_goal[1] - current_pose[1]])
        bearing = np.arctan2(relative_goal_pose[1], relative_goal_pose[0]) - current_pose[2]
        odom_cast = np.concatenate([np.array([self.current_distance / self.max_distance, bearing / np.pi]), current_velocity])
        return odom_cast

    def release(self):
        super().release()
        self.collect_start_point = False
        self.max_distance = None
        self.current_goal = None




"""
Robot State Data Manager
+ Waits for data from all RobotStateData units
+ When all data is acquired, it pushes data further into pipeline
"""

## Brakuje jeszcze przekazywania Goal do OdomStateData, oraz jakiegoś mechanizmu informowania czy wgl zbierać dane czy nie (bo np. epizod nie ruszył)
class RobotStateDataManager:
    def __init__(self, agent_node : Node = None):
        self.agent_node = agent_node
        self.state_data_list : list[RobotStateData] = []
        self.state_data_lock = threading.Lock()
        self.initialize_state_data()

    def open_for_data(self):
        for rsd in self.state_data_list:
            rsd.open_for_data_ = True
            rsd.release()

    def close_for_data(self):
        for rsd in self.state_data_list:
            rsd.open_for_data_ = False
            rsd.release()
    
    def initialize_state_data(self):
        self.lidar_state_data = LidarStateData(self)
        self.odom_state_data = OdomStateData(self)
        self.state_data_list.append(self.lidar_state_data)
        self.state_data_list.append(self.odom_state_data)

    def release_all(self):
        with self.state_data_lock:
            for sd in self.state_data_list:
                sd.release()
    
    def acquired(self, state_data: RobotStateData):
        with self.state_data_lock:
            if all(sd.get_acquired() for sd in self.state_data_list):
                self.process_state_data()
    
    def process_state_data(self):
        ## tutaj może jeszcze inny jakiś processing? Większość procesingu jest i tak robiona w RSD
        processed_data = np.concatenate([sd.get_robot_state_data() for sd in self.state_data_list])
        self.agent_node.predict_action(processed_data)



class RobotActionDataManager:
    def __init__(self, agent_node: Node):
        self.agent_node = agent_node

    def send_action_data(self, action_data):
        msg = Twist()
        msg.linear.x = action_data[0]
        msg.linear.y = action_data[1]
        msg.angular.z = action_data[2]
        self.agent_node.cmd_vel_pub.publish(msg) 
    




