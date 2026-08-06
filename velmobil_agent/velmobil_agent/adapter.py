#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.srv import ControlWorld

class RobotStateData:
    def __init__(self, global_data_acquisitor = None):
        self.robot_state_data_ = None
        self.acquired_ = False
        self.global_data_acquisitor = global_data_acquisitor 
    def release(self):
        self.robot_state_data_ = None
        self.acquired_ = False
    def get_acquired(self):
        return self.acquired_
    def get_robot_state_data(self):
        return self.robot_state_data_
    
    def acquire_data(self, data):
        if not self.acquired_:
            self.robot_state_data_ = self.convert_acquired_data(data)
            self.acquired_ = True
            self.global_data_acquisitor.acquired(self)

    def convert_acquired_data(self, data):
        pass


class LidarStateData(RobotStateData):
    def __init__(self, global_data_acquisitor = None):
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
    def __init__(self, global_data_acquisitor = None):
        super().__init__(global_data_acquisitor)

    # def convert_acquired_data(self, data):
    #     current_pose = np.array([])







## Klasa adaptera ROS-GYM
## Ma ona zawierać metody umożliwające komunikację modelu SB3 z velmobilem
class AdapterDRL:
    def __init__(self):
        





