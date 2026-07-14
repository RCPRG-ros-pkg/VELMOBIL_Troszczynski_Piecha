#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.srv import ControlWorld


class IgnitionConnection(Node):
    def __init__(self, max_retry: int = 10):
        super().__init__('ignition_connection')
        
        self.max_retry_ = max_retry
        self.laser_data_ = np.zeros(60, dtype=np.float32)
        self.current_linear_vel_ = 0.0
        self.done_laser_ = False
        self.done_odom_ = False
        self.reset_sim_done_ = False

        self.action_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/lidar_fusion', self.laser_callback, qos_profile_sensor_data) 
        
        self.world_control_client = self.create_client(ControlWorld, '/world/empty/control')
    
    def publish_velocity(self, velocity):
        msg = Twist()
        msg.linear.x = float(velocity[0])
        msg.linear.y = float(velocity[1])
        msg.angular.z = float(velocity[2])
        self.action_pub.publish(msg)

    def laser_callback(self, msg: LaserScan):
        self.laser_data_ = np.array(msg.ranges)
        self.laser_data_[self.laser_data_ == np.inf] = np.float32(10)
        self.done_laser_ = True
    
    def get_laser_scan(self):
        return self.laser_data_
        
    def odom_callback(self, msg: Odometry):
        self.current_linear_vel_ = msg.twist.twist.linear.x
        self.done_odom_ = True
    
    def get_current_linear_vel(self): 
        return self.current_linear_vel_ 

    def _call_world_control(self, request, description: str):
        while not self.world_control_client.wait_for_service(timeout_sec=self.max_retry_):
            self.get_logger().info(f"Waiting for world control service - {description}...")

        future = self.world_control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"{description} succeeded!")
            return True
        else:
            self.get_logger().error(f"{description} failed: {future.exception()}")
            return False

    def unpause_physics(self):
        request = ControlWorld.Request()
        request.world_control.pause = False
        self._call_world_control(request, "Unpause physics")

    def pause_physics(self):
        request = ControlWorld.Request()
        request.world_control.pause = True
        self._call_world_control(request, "Pause physics")

    def reset_sim(self):
        self.get_logger().info("Resetting simulation")

        request = ControlWorld.Request()
        request.world_control.reset.model_only = True

        success = self._call_world_control(request, "Reset simulation")
        self.reset_sim_done_ = success

        self.unpause_physics()