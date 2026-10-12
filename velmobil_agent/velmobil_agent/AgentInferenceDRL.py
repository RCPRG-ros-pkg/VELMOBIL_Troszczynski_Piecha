#!/usr/bin/env python3
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from .RobotStateData import RobotStateDataManager, RobotActionDataManager
from .Model import Model
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
import time


class AgentInferenceDRL(Node):
    def __init__(self, model: Model = None):
        super().__init__("agent_inference_drl_node")
        self.model = model
        self.latest_state = None
        self.latest_action = None
        self.current_goal_pose = None

        self.goal_tolerance = 0.2

        self.action_done = threading.Event()
        self.initialize_action_utils()
        self.initialize_state_utils()
        self.initialize_nav_action_server()

    def initialize_action_utils(self):
        self.robot_action_data_manager = RobotActionDataManager(self)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def initialize_state_utils(self):
        self.robot_state_data_manager = RobotStateDataManager(self)

        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_fusion", self.robot_state_data_manager.lidar_state_data.acquire_data, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.robot_state_data_manager.odom_state_data.acquire_data, qos_profile_sensor_data)

    def initialize_nav_action_server(self):
        self.nav_callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(self,NavigateToPose, "/navigate_to_pose_drl",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.nav_callback_group,
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(f"New Navigation Goal: {goal_request.pose.pose}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received for navigation goal")
        return CancelResponse.ACCEPT


    # Tutaj multithreaded executor jak będzie ktoś z nas pisał main pod to
    def execute_callback(self, goal_handle):
        goal_pose = goal_handle.request.pose
        self.current_goal_pose = goal_pose.pose
        self.robot_state_data_manager.odom_state_data.set_current_goal(np.array([self.current_goal_pose.position.x, self.current_goal_pose.position.y, self.current_goal_pose.position.z]))
        self.robot_state_data_manager.open_for_data()
        start_time = time.time()
        feedback_msg = NavigateToPose.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                self.get_logger().info("Goal Canceled")
                return NavigateToPose.Result()


            if not self.next_step(timeout=3.0): # release all -> collect new data -> RDSM::acquired -> RDSM::process_state_data -> AgentDRL::predict_action -> RADM::send_action_data
                continue

            distance_remaining = self.robot_state_data_manager.odom_state_data.current_distance
            if distance_remaining < self.goal_tolerance:
                goal_handle.succeed()
                self.stop_robot()
                self.get_logger().info("Goal Reached")
                return NavigateToPose.Result()

            # feedback
            feedback_msg.distance_remaining = float(distance_remaining)
            feedback_msg.current_pose = self.current_pose_stamped()
            elapsed = time.time() - start_time
            feedback_msg.navigation_time.sec = int(elapsed)
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.abort()
        return NavigateToPose.Result()


    def stop_robot(self):
        self.robot_state_data_manager.close_for_data()
        self.cmd_vel_pub.publish(Twist())


    def predict_action(self, data):
        self.latest_state = data
        action = self.model.predict(data)
        self.robot_action_data_manager.send_action_data(action)
        self.action_done.set()

    def next_step(self, timeout=None):
        self.action_done.clear()
        self.robot_state_data_manager.release_all()
        return self.action_done.wait(timeout=timeout)