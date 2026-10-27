#!/usr/bin/env python3
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from .RobotStateData import RobotStateDataManager, RobotActionDataManager
from .Model import Model
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from velmobil_interfaces.srv import InferenceScenario
import time
from velmobil_agent.utilities.logs_creators import save_trial_outputs, evaluation_summary, record_outcome_elapsed_time


"""
Agent Inference DRL
+ Loads trained policy (as Model class) and exposes it as NavigateToPose action server
+ Observation is built exactly the same way as in AgentTrainingDRL:
 90 (processed-lidar) + 5 (norm-distance, norm-bearing, vx, vy, omega)
+ Before each goal asks SimulationManager (/inference_scenario) to prepare stage, start and goal
+ Evaluation iterates over setted stages
+ Settings are read from inference_params.yaml
"""

LIDAR_TIMESTAMP_DIFFERENCE = None
ODOM_TIMESTAMP_DIFFERENCE = None


class AgentInferenceDRL(Node):
    def __init__(self, model: Model = None, evaluation_save_path: str = None):
        super().__init__("agent_inference_drl_node", automatically_declare_parameters_from_overrides = True)
        self.model = model
        self.evaluation_save_path = evaluation_save_path

        self.max_navigation_time = self.get_parameter("max_navigation_time").value
        self.abort_on_collision = self.get_parameter("abort_on_collision").value
        self.collision_range = self.get_parameter("collision_range").value

        self.start_position = np.array([0.0, 0.0])
        self.goal_position = np.array([0.0, 0.0])
        self.goal_tolerance = 0.5

        self.done_evaluation = threading.Event()
        self.trial_done_event = threading.Event()

        self.action_done = threading.Event()
        self.inference_scenario_event = threading.Event()

        self.initialize_action_utils()
        self.initialize_state_utils()
        self.initialize_simulation_utils()
        self.initialize_nav_action_server()
        self.initialize_nav_action_client()
        self.initialize_evaluation_utils()

        #   Zamiast timera dodano wątek - timer na use_sim_time nie ruszy bez /clock
        threading.Thread(target=self.run_evaluation, daemon=True).start()

    def initialize_action_utils(self):
        self.latest_action = None
        self.active_goal_handle = None
        self.current_goal_pose = None
        self.robot_action_data_manager = RobotActionDataManager(self)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def initialize_state_utils(self):
        self.latest_full_state = None
        self.latest_state = None
        self.robot_state_data_manager = RobotStateDataManager(self, LIDAR_TIMESTAMP_DIFFERENCE, ODOM_TIMESTAMP_DIFFERENCE)
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_fusion", self.robot_state_data_manager.lidar_state_data.acquire_data, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, "/model/velmobil/odometry_with_covariance", self.robot_state_data_manager.odom_state_data.acquire_data, 10)

    def initialize_simulation_utils(self):
        self.simulation_callback_group = ReentrantCallbackGroup()
        self.inference_scenario_client = self.create_client(InferenceScenario, '/inference_scenario', callback_group=self.simulation_callback_group)

    def initialize_nav_action_server(self):
        self.nav_callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(self, NavigateToPose, "/navigate_to_pose_drl",
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback,
            callback_group = self.nav_callback_group
        )

    def initialize_nav_action_client(self):
        self.nav_client_callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose_drl", callback_group=self.nav_client_callback_group)

    def initialize_evaluation_utils(self):
        self.stage = ""
        self.trial = 0
        self.stage_parameters = {}
        self.evaluation_results = []
        self.trial_status = None
        self.trial_outcome = "unknown"
        self.trial_time = 0.0
        self.inference_scenario_success = False
        self.finished_evaluation = False

    def run_evaluation(self):
        time.sleep(2.0)
        if not self._action_client.wait_for_server(timeout_sec=10.0) or not self.inference_scenario_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Action server /navigate_to_pose_drl or service /inference_scenario not available")
            self.done_evaluation.set()
            return
        while rclpy.ok():
            self.reset_trial()
            if not self.request_inference_scenario():
                self.get_logger().error("[AgentInferenceDRL] Timeout waiting for inference scenario")
                break
            if self.finished_evaluation:
                break
            if self.inference_scenario_success:
                self.send_goal_and_wait()
            else:
                self.trial_outcome = "scenario_failed"
            save_trial_outputs(self.stage, self, self.evaluation_results, self.evaluation_save_path)
            self.get_logger().info(f"[AgentInferenceDRL] Evaluation stage: {self.stage} | Trial: {self.trial} | Outcome: {self.trial_outcome} | Time: {self.trial_time:.2f}s")
        evaluation_summary(self)
        self.done_evaluation.set()

    def reset_trial(self):
        self.trial_done_event.clear()
        self.trial_status = None
        self.trial_outcome = "unknown"
        self.trial_time = 0.0

    def send_goal_and_wait(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "odom"
        goal.pose.pose.position.x = float(self.goal_position[0])
        goal.pose.pose.position.y = float(self.goal_position[1])
        goal.pose.pose.orientation.w = 1.0
        future = self._action_client.send_goal_async(goal, feedback_callback = self.goal_feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        if not self.trial_done_event.wait(timeout=self.max_navigation_time + 240.0):
            self.get_logger().error("[AgentInferenceDRL] Timeout waiting for trial result")
            self.trial_outcome = "no_result"

    def goal_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        pose = feedback.current_pose.pose.position
        self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f}m, Pose: [{pose.x:.2f}; {pose.y:.2f}], Time: {feedback.navigation_time.sec}s", throttle_duration_sec=0.5)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.trial_outcome = "rejected"
            self.trial_done_event.set()
            return
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.trial_status = future.result().status
        status_names = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED", GoalStatus.STATUS_CANCELED: "CANCELED"}
        self.get_logger().info(f"Goal finished: {status_names.get(self.trial_status, self.trial_status)}")
        self.trial_done_event.set()

    def goal_callback(self, goal_request):
        if self.model is None:
            self.get_logger().error("No model loaded - rejecting goal")
            return GoalResponse.REJECT
        self.get_logger().info(f"New navigation goal: {goal_request.pose.pose}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received for navigation goal")
        return CancelResponse.ACCEPT

    def request_inference_scenario(self):
        self.inference_scenario_event.clear()
        self.inference_scenario_success = False
        self.finished_evaluation = False
        future = self.inference_scenario_client.call_async(InferenceScenario.Request())
        future.add_done_callback(self.setup_inference_scenario_callback)
        return self.inference_scenario_event.wait(timeout=180.0)

    def setup_inference_scenario_callback(self, future):
        result = future.result()
        self.inference_scenario_success = result is not None and result.success
        self.finished_evaluation = result is not None and result.done_evaluation
        if result is not None and not self.finished_evaluation:
            self.stage = result.stage
            self.trial = result.trial
            self.goal_tolerance = result.goal_tolerance
            self.stage_parameters = {
                "static_obstacles_num": result.static_obstacles_num,
                "dynamic_obstacles_num": result.dynamic_obstacles_num,
                "goal_distance": result.goal_distance,
            }
        if self.inference_scenario_success and not self.finished_evaluation:
            self.get_logger().info(f"{result.message}")
            self.start_position = np.array([result.initial_pos_x, result.initial_pos_y])
            self.goal_position = np.array([result.goal_pos_x, result.goal_pos_y])
        elif not self.inference_scenario_success:
            self.get_logger().error(f"Couldn't setup inference scenario: {result.message if result else 'no response'}")
        self.inference_scenario_event.set()

    def execute_callback(self, goal_handle):
        self.active_goal_handle = goal_handle
        goal_pose = goal_handle.request.pose
        self.current_goal_pose = goal_pose.pose
        self.goal_position = np.array([self.current_goal_pose.position.x, self.current_goal_pose.position.y])
        self.stop_robot()
        time.sleep(0.5)
        self.robot_state_data_manager.zero_last_acquired_timestamp_all()
        self.robot_state_data_manager.odom_state_data.set_current_goal(self.goal_position)
        self.robot_state_data_manager.open_for_data()
        start_time = time.time()
        feedback_msg = NavigateToPose.Feedback()
        while rclpy.ok():
            if self.active_goal_handle is not goal_handle:
                goal_handle.abort()
                self.get_logger().info("Goal Preempted by a new goal")
                return NavigateToPose.Result()
            if goal_handle.is_cancel_requested:
                self.trial_outcome = "canceled"
                goal_handle.canceled()
                self.stop_robot()
                self.get_logger().info("Goal Canceled")
                return NavigateToPose.Result()
            elapsed = time.time() - start_time
            if self.max_navigation_time > 0.0 and elapsed > self.max_navigation_time:
                record_outcome_elapsed_time(self, "timeout", elapsed)
                goal_handle.abort()
                self.stop_robot()
                self.get_logger().warn(f"Goal Aborted - navigation time exceeded {self.max_navigation_time:.1f}s")
                return NavigateToPose.Result()
            if not self.next_step(timeout = 3.0):
                self.get_logger().warn("Timeout waiting for state data", throttle_duration_sec=2.0)
                continue
            if self.latest_full_state["odom"]["current_distance"] < self.goal_tolerance:
                record_outcome_elapsed_time(self, "reached", elapsed)
                goal_handle.succeed()
                self.stop_robot()
                self.get_logger().info(f"Goal Reached in {elapsed:.2f}s")
                return NavigateToPose.Result()
            if self.abort_on_collision and bool(np.any(self.latest_full_state["lidar"]["raw_ranges"] < self.collision_range)):
                record_outcome_elapsed_time(self, "collision", elapsed)
                goal_handle.abort()
                self.stop_robot()
                self.get_logger().warn(f"Goal Aborted - obstacle closer than {self.collision_range:.2f}m")
                return NavigateToPose.Result()
            feedback_msg.distance_remaining = float(self.latest_full_state["odom"]["current_distance"])
            feedback_msg.current_pose = self.current_pose_stamped(self.latest_full_state["odom"]["current_pose"])
            feedback_msg.navigation_time.sec = int(elapsed)
            feedback_msg.navigation_time.nanosec = int((elapsed % 1.0)*1e9)
            goal_handle.publish_feedback(feedback_msg)
        goal_handle.abort()
        self.stop_robot()
        return NavigateToPose.Result()

    def current_pose_stamped(self, current_pose) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.pose.position.x = float(current_pose[0])
        pose.pose.position.y = float(current_pose[1])
        pose.pose.orientation.z = float(np.sin(current_pose[2] / 2.0))
        pose.pose.orientation.w = float(np.cos(current_pose[2] / 2.0))
        return pose

    def stop_robot(self):
        self.robot_state_data_manager.close_for_data()
        self.cmd_vel_pub.publish(Twist())

    def predict_action(self, data):
        self.latest_full_state = data
        self.latest_state = np.concatenate([data["lidar"]["processed_ranges"], data["odom"]["normalized_distance"]]).astype(np.float32)
        self.latest_action = self.model.predict(self.latest_state, deterministic = True)
        self.robot_action_data_manager.send_action_data(self.latest_action)
        self.action_done.set()

    def next_step(self, timeout=None):
        self.action_done.clear()
        self.robot_state_data_manager.release_all()
        return self.action_done.wait(timeout = timeout)