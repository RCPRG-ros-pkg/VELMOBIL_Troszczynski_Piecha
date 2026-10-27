#!/usr/bin/env python3
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from velmobil_interfaces.srv import StageModification, InferenceScenario
from std_srvs.srv import Trigger
from dataclasses import dataclass
from gz.transport13 import Node as GZNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from rclpy.clock import Clock, ClockType
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.twist_pb2 import Twist as GZTwist


"""
Simulation Manager
+ responsible for spawning and configuring simulation objects by low level component ObjectController.
+ ObjectController class talk with Gazebo directly via gz_transport bindings
+ training : /quick_reset_simulation, /hard_reset_simulation
+ inference : /inference_scenario
"""

@dataclass(frozen=True)
class StageSettings:
    static_obstacles_num: int
    dynamic_obstacles_num: int
    min_obstacle_speed: float
    max_obstacle_speed: float
    goal_distance: float = 0.0
    goal_tolerance: float = 0.0


def sdf_reader(path: str) -> str:
    with open(path, 'r') as file_handler:
        return file_handler.read()


class ObjectController:
    def __init__(self, stage_settings: StageSettings, logger):
        self._world_name = 'velmobil_arena'
        self._logger = logger
        self._gz_node = GZNode()

        self._static_rect_obstacle_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'static_obstacle_rect.sdf'))
        self._dynamic_rect_obstacle_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'dynamic_obstacles', 'dynamic_obstacle_rect.sdf'))
        self._start_object_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'start_object.sdf'))
        self._goal_object_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'goal_object.sdf'))

        self.set_stage_settings(stage_settings)
        self.static_obstacle_positions = []
        self._spawned_static = []
        self._spawned_dynamic = []
        self._dynamic_publishers = {}
        self._dynamic_velocities = {}

    def set_stage_settings(self, stage_settings: StageSettings):
        self._static_obstacles_num = stage_settings.static_obstacles_num
        self._dynamic_obstacles_num = stage_settings.dynamic_obstacles_num
        self._min_obstacle_speed = stage_settings.min_obstacle_speed
        self._max_obstacle_speed = stage_settings.max_obstacle_speed

    def create_object(self, name: str, sdf: str, position, yaw: float = 0.0) -> bool:
        req = EntityFactory()
        req.name = name
        req.sdf = sdf
        req.allow_renaming = False
        req.pose.position.x = float(position[0])
        req.pose.position.y = float(position[1])
        req.pose.position.z = float(position[2])
        req.pose.orientation.z = float(np.sin(yaw / 2.0))
        req.pose.orientation.w = float(np.cos(yaw / 2.0))
        result, response = self._gz_node.request(f'/world/{self._world_name}/create', req, EntityFactory, Boolean, 10000)
        if not (result and response.data):
            self._logger.warn(f"Failed spawn for {name}")
            return False
        return True

    def remove_object(self, name: str) -> bool:
        req = Entity()
        req.name = name
        req.type = Entity.MODEL
        result, response = self._gz_node.request(f'/world/{self._world_name}/remove', req, Entity, Boolean, 5000)
        if not (result and response.data):
            self._logger.warn(f"Failed object removing for {name}")
            return False
        return True

    def set_entity_pose(self, name: str, position: np.ndarray, z: float = 0.0) -> bool:
        req = Pose()
        req.name = name
        req.position.x = float(position[0])
        req.position.y = float(position[1])
        req.position.z = float(z)
        req.orientation.w = 1.0
        result, response = self._gz_node.request(f'/world/{self._world_name}/set_pose', req, Pose, Boolean, 5000)
        if not result:
            self._logger.warn(f"Set_pose request failed/timed out for {name}")
            return False
        return response.data

    def spawn_static_obstacles(self, area_size: float) -> None:
        positions = []
        while len(positions) < self._static_obstacles_num:
            position = (np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2))
            if position in positions:
                continue
            positions.append(position)
        self.static_obstacle_positions = positions
        for i, (x, y) in enumerate(positions):
            name = f'static_obstacle_{i}'
            if self.create_object(name, self._static_rect_obstacle_sdf, (x, y, 1.1)):
                self._spawned_static.append(name)

    def spawn_dynamic_obstacles(self, area_size: float) -> None:
        positions_orientations = [(-np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2),np.random.uniform(-np.pi, np.pi)) for _ in range(self._dynamic_obstacles_num)]
        speeds = np.random.uniform(self._min_obstacle_speed, self._max_obstacle_speed, size=len(positions_orientations))
        for i, ((x, y, yaw), speed) in enumerate(zip(positions_orientations, speeds)):
            name = f'dynamic_obstacle_{i}'
            topic = f'/model/{name}/cmd_vel'
            sdf = self._dynamic_rect_obstacle_sdf.replace('/model/dynamic_obstacle_rectangle/cmd_vel', topic)
            if not self.create_object(name, sdf, (x, y, 1.1), yaw):
                continue
            self._spawned_dynamic.append(name)
            if name not in self._dynamic_publishers:
                self._dynamic_publishers[name] = self._gz_node.advertise(topic, GZTwist)
            msg = GZTwist()
            msg.linear.x = float(speed)
            self._dynamic_velocities[name] = msg

    def remove_static_obstacles(self) -> None:
        for name in self._spawned_static:
            self.remove_object(name)
        self._spawned_static.clear()
        self.static_obstacle_positions = []

    def remove_dynamic_obstacles(self) -> None:
        for name in self._spawned_dynamic:
            self.remove_object(name)
            self._dynamic_velocities.pop(name, None)
        self._spawned_dynamic.clear()

    def set_velocity_for_dynamic_obstacles(self) -> None:
        for name, msg in self._dynamic_velocities.items():
            self._dynamic_publishers[name].publish(msg)

    def spawn_start_object(self, start_coordinates: np.ndarray) -> None:
        self.create_object('start_object', self._start_object_sdf, (start_coordinates[0], start_coordinates[1], 0.0))

    def spawn_goal_object(self, goal_coordinates: np.ndarray) -> None:
        self.create_object('goal_object', self._goal_object_sdf, (goal_coordinates[0], goal_coordinates[1], 0.0))
    
    def move_start_object(self, start: np.ndarray) -> bool:
        return self.set_entity_pose('start_object', start)

    def move_goal_object(self, goal: np.ndarray) -> bool:
        return self.set_entity_pose('goal_object', goal)

    def teleport_robot(self, position: np.ndarray) -> bool:
        return self.set_entity_pose('velmobil', position, z=0.1)

    
class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager', automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("[SimulationManager] initialization")
        self.arena_size = self.get_parameter("arena_size").value
        self.seed = self.parameter("seed")
        if self.seed is not None:
            np.random.seed(self.seed)

        self.stage_names = list(self.get_parameter("stage_names").value)
        self.stage_settings = {name: StageSettings(**{k: p.value for k, p in self.get_parameters_by_prefix(f"stages.{name}").items()}) for name in self.stage_names}
        self.current_stage_name = self.stage_names[0]
        self.get_logger().info(f"[SimulationManager] Acquired {len(self.stage_settings)} stage settings")

        self.trials_per_stage = self.parameter("trials_per_stage", 0)
        self.random_start_goal = self.parameter("random_start_goal", True)
        self.inference_start_position = np.array(self.parameter("start_position", [0.0, 0.0]), dtype=float)
        self.inference_goal_position = np.array(self.parameter("goal_position", [0.0, 0.0]), dtype=float)
        self.reroll_obstacles_each_trial = self.parameter("reroll_obstacles_each_trial", False)
        self._inference_stage_index = 0
        self._inference_trial = 0

        self.start_position = np.array([0.0, 0.0])
        self.goal_position = np.array([0.0, 0.0])

        self._object_controller = ObjectController(self.stage_settings[self.current_stage_name], self.get_logger())
        self._object_controller.spawn_start_object(self.start_position)
        self._object_controller.spawn_goal_object(self.goal_position)
        self.get_logger().info(f"[SimulationManager] Initialized Object Controller for {self.current_stage_name}")

        self._dynamic_obstacles_timer = self.create_timer(0.5, self._object_controller.set_velocity_for_dynamic_obstacles, clock = Clock(clock_type = ClockType.STEADY_TIME))

        self._quick_reset_service = self.create_service(Trigger, '/quick_reset_simulation', self.handle_quick_reset_request)
        self._hard_reset_service = self.create_service(StageModification, '/hard_reset_simulation', self.handle_hard_reset_request)
        self._inference_scenario_service = self.create_service(InferenceScenario, '/inference_scenario', self.handle_inference_scenario_request)

    def parameter(self, name: str, default_parameter = None):
        return self.get_parameter(name).value if self.has_parameter(name) else default_parameter

    def change_stage(self, stage_name: str) -> None:
        self._object_controller.remove_static_obstacles()
        self._object_controller.remove_dynamic_obstacles()
        self.current_stage_name = stage_name
        self._object_controller.set_stage_settings(self.stage_settings[self.current_stage_name])
        self._object_controller.spawn_dynamic_obstacles(self.arena_size)
        self._object_controller.spawn_static_obstacles(self.arena_size)

    def is_clear_of_static_obstacles(self, start, goal, clearence: float = 1.5) -> bool:
        obstacles = np.array(self._object_controller.static_obstacle_positions, dtype = float).reshape(-1, 2)
        return bool(np.all(np.linalg.norm(obstacles - start, axis = 1) > clearence) and np.all(np.linalg.norm(obstacles - goal, axis = 1) > clearence))

    def set_start_goal_position(self, max_attempts: int = 10000) -> bool:
        for _ in range(max_attempts):
            start = np.random.uniform(-(self.arena_size - 1) / 2, (self.arena_size - 1) / 2, size=(2,))
            direction = np.random.uniform(-1.0, 1.0, size=(2,))
            goal = self.stage_settings[self.current_stage_name].goal_distance * (direction / np.linalg.norm(direction)) + start
            if np.all(np.abs(goal) > ((self.arena_size - 1) / 2)):
                continue
            if self.is_clear_of_static_obstacles(start, goal):
                self.start_position, self.goal_position = start, goal
                return True
        return False

    def apply_start_goal(self) -> None:
        if not self._object_controller.teleport_robot(self.start_position):
            self.get_logger().warn("Robot not moved")
        if not self._object_controller.move_goal_object(self.goal_position):
            self.get_logger().warn("Goal object not moved")
        if not self._object_controller.move_start_object(self.start_position):
            self.get_logger().warn("Start object not moved")

    def fill_start_goal_response(self, response) -> None:
        response.initial_pos_x = float(self.start_position[0])
        response.initial_pos_y = float(self.start_position[1])
        response.goal_pos_x = float(self.goal_position[0])
        response.goal_pos_y = float(self.goal_position[1])
        response.goal_tolerance = self.stage_settings[self.current_stage_name].goal_tolerance

    def handle_quick_reset_request(self, request, response):
        self.get_logger().info("handle quick reset simulation")
        self.apply_start_goal()
        response.success = True
        response.message = "\n| Simulation quick reset executed successfully\n"
        return response

    def handle_hard_reset_request(self, request, response):
        self.get_logger().info("handle hard reset simulation")
        if request.change_stage != 0:
            index = self.stage_names.index(self.current_stage_name)
            match request.change_stage:
                case request.STAGE_UP:
                    index = min(index + 1, len(self.stage_names) - 1)
                case request.STAGE_DOWN:
                    index = max(index - 1, 0)
            self.change_stage(self.stage_names[index])
        if not self.set_start_goal_position():
            response.success = False
            response.message = "Couldn't find valid start & goal"
            return response
        self.apply_start_goal()
        self.fill_start_goal_response(response)
        response.success = True
        response.message = f"\n| Current Stage Level: {self.current_stage_name}\n"
        response.message += "| New Stage: Yes\n" if request.change_stage != 0 else "| New Stage: No\n"
        response.message += f"| Current Start: [{self.start_position[0]}; {self.start_position[1]}]\n"
        response.message += f"| Current Goal: [{self.goal_position[0]}; {self.goal_position[1]}]\n"
        return response

    def handle_inference_scenario_request(self, request, response):
        self.get_logger().info("handle inference scenario")
        if self._inference_trial >= self.trials_per_stage:
            self._inference_trial = 0
            self._inference_stage_index += 1
        if self._inference_stage_index >= len(self.stage_names):
            response.done_evaluation = True
            response.success = True
            response.message = "\n| Inference evaluation finished\n"
            return response
        if self._inference_trial == 0:
            if self.seed is not None:
                np.random.seed(self.seed + self._inference_stage_index)
            self.change_stage(self.stage_names[self._inference_stage_index])
        elif self.reroll_obstacles_each_trial:
            self.change_stage(self.stage_names[self._inference_stage_index])
        self._inference_trial += 1
        response.stage = self.current_stage_name
        response.trial = self._inference_trial
        response.goal_distance = self.stage_settings[self.current_stage_name].goal_distance
        response.static_obstacles_num = self.stage_settings[self.current_stage_name].static_obstacles_num
        response.dynamic_obstacles_num = self.stage_settings[self.current_stage_name].dynamic_obstacles_num
        if self.random_start_goal:
            valid = self.set_start_goal_position()
        else:
            self.start_position = self.inference_start_position.copy()
            self.goal_position = self.inference_goal_position.copy()
            valid = self.is_clear_of_static_obstacles(self.start_position, self.goal_position)
        if not valid:
            response.success = False
            response.message = "Couldn't find start & goal of static obstacles"
            return response
        self.apply_start_goal()
        self.fill_start_goal_response(response)
        response.success = True
        response.message = f"\n| Inference stage: {self.current_stage_name} | Trial: {self._inference_trial}/{self.trials_per_stage}\n"
        response.message += f"| Start: [{self.start_position[0]}; {self.start_position[1]}]\n"
        response.message += f"| Goal: [{self.goal_position[0]}; {self.goal_position[1]}]\n"
        response.message += f"| Goal tolerance: {response.goal_tolerance}\n"
        return response


def main():
    rclpy.init()
    simulation_manager = SimulationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(simulation_manager)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        simulation_manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()