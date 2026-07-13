import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class IgnitionConnection(Node):
    def __init__(self, max_retry: int = 10):
        super().__init__('ignition_connection')
        
        self.max_retry_ = max_retry
        
        self.action_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/lidar_fusion', self.laser_callback, 10) 
        
        self.unpause_sim_client = self.create_client(Empty, '/unpause_physics')
        self.pause_sim_client = self.create_client(Empty, '/pause_physics')
        self.reset_sim_client = self.create_client(Empty, '/reset_simulation')
        self.reset_world_client = self.create_client(Empty, '/reset_world') 
    
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
        return  self.current_linear_vel_ 
    
    def reset_sim(self):
        self.get_logger().info("Resetting simulation")
        
        while not self.reset_sim_client.wait_for_service(timeout_sec=self.max_retry_):
            self.get_logger().info("Waiting for reset simulation service...")

        self.get_logger().info("Reset simulation service found!")
        
        request = Empty.Request()
        future = self.reset_sim_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info("Simulation successfully reset!")
            self.reset_sim_done_ = True
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")
    
