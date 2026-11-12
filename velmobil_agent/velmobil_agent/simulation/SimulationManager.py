#!/usr/bin/env python3
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from threading import Event
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from velmobil_interfaces.srv import StageModification, InferenceScenario, SetPose
from std_srvs.srv import Trigger
from dataclasses import dataclass
from gz.transport13 import Node as GZNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from rclpy.clock import Clock, ClockType
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.twist_pb2 import Twist as GZTwist
from velmobil_agent.occupancy_grid.OccupancyGrid import Obstacle, OccupancyGrid
from velmobil_agent.occupancy_grid.blender_utilities import blender_obj_parser, get_path_to_obj_map, create_sdf_from_obj
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy


"""
Simulation Manager
+ responsible for spawning and configuring simulation objects by low level component ObjectController.
+ ObjectController class talk with Gazebo directly via gz_transport bindings
+ training : /quick_reset_simulation, /hard_reset_simulation
+ inference : /inference_scenario
"""

@dataclass(frozen=True)
class StageSettings:
    map: str
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
        self._spawn_generation = 0
        self._stage_settings = stage_settings

        self._static_rect_obstacle_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'static_obstacle_rect.sdf'))
        self._dynamic_rect_obstacle_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'dynamic_obstacles', 'dynamic_obstacle_rect.sdf'))
        self._start_object_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'start_object.sdf'))
        self._goal_object_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'goal_object.sdf'))
        self._free_voxel_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'free_voxel.sdf'))
        self._occupied_voxel_sdf = sdf_reader(os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'occupied_voxel.sdf'))
        
        self.set_stage_settings(stage_settings)
        self.static_obstacle_positions = []
        
        self.main_occupancy_grid = OccupancyGrid(0.25, np.array([30.0, 30.0]), np.array([-15.0, -15.0]))
        self.static_map_occupancy_grid = OccupancyGrid(0.25, np.array([30.0, 30.0]), np.array([-15.0, -15.0]))
        self.static_obstacles_occupancy_grid = OccupancyGrid(0.25, np.array([30.0, 30.0]), np.array([-15.0, -15.0]))
        
        self.spawned_map_name: str = None
        self._spawned_static = []
        self._spawned_dynamic = []
        self._dynamic_publishers = {}
        self._dynamic_velocities = {}


    def update_and_inflate_main_occupancy_grid(self) -> None:
        self.main_occupancy_grid = self.static_map_occupancy_grid + self.static_obstacles_occupancy_grid
        self.main_occupancy_grid.inflate(3)

    def set_stage_settings(self, stage_settings: StageSettings):
        self._map = stage_settings.map
        self._static_obstacles_num = stage_settings.static_obstacles_num
        self._dynamic_obstacles_num = stage_settings.dynamic_obstacles_num
        self._min_obstacle_speed = stage_settings.min_obstacle_speed
        self._max_obstacle_speed = stage_settings.max_obstacle_speed

    def create_object(self, name: str, sdf: str, position, yaw: float = 0.0, add_to_static_grid: bool = True) -> bool:
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

        if add_to_static_grid:
            self.static_obstacles_occupancy_grid.place_obstacle(Obstacle(name, float(position[0]), float(position[1]), 1.0, 1.0))
        return True

    

    def remove_object(self, name: str) -> bool:
        req = Entity()
        req.name = name
        req.type = Entity.MODEL
        result, response = self._gz_node.request(f'/world/{self._world_name}/remove', req, Entity, Boolean, 5000)
        if not (result and response.data):
            self._logger.warn(f"Failed object removing for {name}")
            return False
        self.static_obstacles_occupancy_grid.remove_obstacle(name)
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
    
    def remove_leftover_obstacles(self) -> None:
        result, scene = self._gz_node.request(f'/world/{self._world_name}/scene/info', Empty(), Empty, Scene, 5000)
        if not result:
            self._logger.warn("Couldn't read scene info - leftover obstacles not removed")
            return
        for model in scene.model:
            if model.name.startswith(('static_obstacle_', 'dynamic_obstacle_')):
                self.remove_object(model.name)

    def spawn_world(self, map_name: str) -> None:
        if self.spawned_map_name == map_name:
            return
        if self.spawned_map_name is not None:
            self.remove_world()
        map_sdf = create_sdf_from_obj(map_name)
        if not self.create_object(map_name, map_sdf, (0.0, 0.0, 0.0), 0.0, add_to_static_grid=False):
            return
        self.spawned_map_name = map_name
        obstacles = blender_obj_parser(get_path_to_obj_map(map_name))
        self.static_map_occupancy_grid.clear()
        self.static_map_occupancy_grid.place_obstacles(obstacles)

    def remove_world(self) -> None:
        if self.spawned_map_name is None:
            return
        self.remove_object(self.spawned_map_name)
        self.spawned_map_name = None
        self.static_map_occupancy_grid.clear()

    def spawn_static_obstacles(self, area_size: float) -> None:
        self._spawn_generation += 1
        positions = []
        while len(positions) < self._stage_settings.static_obstacles_num:
            position = (np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2))
            if position in positions:
                continue
            positions.append(position)
        self.static_obstacle_positions = positions
        for i, (x, y) in enumerate(positions):
            name = f'static_obstacle_{self._spawn_generation}_{i}'
            if self.create_object(name, self._static_rect_obstacle_sdf, (x, y, 1.1)):
                self._spawned_static.append(name)

    def spawn_dynamic_obstacles(self, area_size: float) -> None:
        self._spawn_generation += 1
        positions_orientations = [(-np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2),np.random.uniform(-np.pi, np.pi)) for _ in range(self._stage_settings.dynamic_obstacles_num)]
        speeds = np.random.uniform(self._stage_settings.min_obstacle_speed, self._stage_settings.max_obstacle_speed, size=len(positions_orientations))
        for i, ((x, y, yaw), speed) in enumerate(zip(positions_orientations, speeds)):
            name = f'dynamic_obstacle_{self._spawn_generation}_{i}'
            topic = f'/model/dynamic_obstacle_{i}/cmd_vel'
            sdf = self._dynamic_rect_obstacle_sdf.replace('/model/dynamic_obstacle_rectangle/cmd_vel', topic)
            if not self.create_object(name, sdf, (x, y, 1.1), yaw):
                continue
            self._spawned_dynamic.append(name)
            if topic not in self._dynamic_publishers:
                self._dynamic_publishers[topic] = self._gz_node.advertise(topic, GZTwist)
            msg = GZTwist()
            msg.linear.x = float(speed)
            self._dynamic_velocities[topic] = msg

    def remove_obstacles(self) -> None:
        for name in (self._spawned_static + self._spawned_dynamic):
            self.remove_object(name)
        self._spawned_static.clear()
        self.static_obstacle_positions = []
        self.static_obstacles_occupancy_grid.clear()

    def remove_dynamic_obstacles(self) -> None:
        for name in self._spawned_dynamic:
            self.remove_object(name)
            self._dynamic_velocities.pop(name, None)
        self._spawned_dynamic.clear()
        self.static_obstacle_positions = []
        self._dynamic_velocities.clear()

    def set_velocity_for_dynamic_obstacles(self) -> None:
        for topic, msg in self._dynamic_velocities.items():
            self._dynamic_publishers[topic].publish(msg)

    def spawn_markers(self, start_coordinates: np.ndarray, goal_coordinates: np.ndarray) -> None:
        self.create_object('start_object', self._start_object_sdf, (start_coordinates[0], start_coordinates[1], 0.0))
        self.create_object('goal_object', self._goal_object_sdf, (goal_coordinates[0], goal_coordinates[1], 0.0))

    def place_markers_and_robot(self, start_coordinates: np.ndarray, goal_coordinates: np.ndarray) -> bool:
        robot_moved = self.set_entity_pose('velmobil', start_coordinates, z = 0.1)
        start_moved = self.set_entity_pose('start_object', start_coordinates)
        goal_moved = self.set_entity_pose('goal_object', goal_coordinates)
        return robot_moved and start_moved and goal_moved

    
class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager', automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("[SimulationManager] initialization")
        self.arena_size = self.get_parameter("arena_size").value
        self.seed = self.parameter("seed")
        self.__set_pose_controller_event = Event()
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
        self._object_controller.remove_leftover_obstacles()
        self._object_controller.spawn_markers(self.start_position, self.goal_position)
        self.get_logger().info(f"[SimulationManager] Initialized Object Controller for {self.current_stage_name}")

        self._dynamic_obstacles_timer = self.create_timer(0.5, self._object_controller.set_velocity_for_dynamic_obstacles, clock = Clock(clock_type = ClockType.STEADY_TIME))

        self._quick_reset_service = self.create_service(Trigger, '/quick_reset_simulation', self.handle_quick_reset_request)
        self._hard_reset_service = self.create_service(StageModification, '/hard_reset_simulation', self.handle_hard_reset_request)
        self._roll_yaw_controller_reset = self.create_client(SetPose, '/roll_yaw_controller/set_pose')
        self._inference_scenario_service = self.create_service(InferenceScenario, '/inference_scenario', self.handle_inference_scenario_request)
        self._occupancy_grid_publisher = self.create_publisher(OccupancyGridMsg, '/occupancy_grid', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._start_goal_markers_publisher = self.create_publisher(MarkerArray, '/start_goal_markers', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
  

    def publish_occupancy_grid(self) -> None:
        grid = self._object_controller.main_occupancy_grid
        data = grid.inflated_grid[::-1, ::-1].T # numpy index -> standard index [y, x], as expected by nav_msgs
        msg = OccupancyGridMsg()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(grid.voxel_size)
        msg.info.width = data.shape[1]
        msg.info.height = data.shape[0]
        msg.info.origin.position.x = float(grid.map_origin[0])
        msg.info.origin.position.y = float(grid.map_origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = np.where(data != 0, 100, 0).astype(np.int8).flatten().tolist()
        self._occupancy_grid_publisher.publish(msg)

    def publish_start_goal_markers(self) -> None:
        markers_data = [
            (0, 'start', self.start_position, (0.85, 0.9, 0.1)),
            (1, 'goal', self.goal_position, (0.0, 0.8, 0.0)),
        ]
        marker_array = MarkerArray()
        for marker_id, namespace, position, (r, g, b) in markers_data:
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self._start_goal_markers_publisher.publish(marker_array)

    def parameter(self, name: str, default_parameter = None):
        return self.get_parameter(name).value if self.has_parameter(name) else default_parameter

    def change_stage(self, stage_name: str) -> None:
        self._object_controller.remove_obstacles()
        self.current_stage_name = stage_name
        self._object_controller.set_stage_settings(self.stage_settings[self.current_stage_name])
        self._object_controller.spawn_dynamic_obstacles(self.arena_size)
        self._object_controller.spawn_static_obstacles(self.arena_size)
        self._object_controller.spawn_world(self.stage_settings[self.current_stage_name].map)
        self._object_controller.update_and_inflate_main_occupancy_grid()
        self.publish_occupancy_grid()

    def is_clear_of_static_obstacles(self, start, goal, clearence: float = 1.5) -> bool:
        start_clear = self._object_controller.main_occupancy_grid.is_free(start[0], start[1])
        goal_clear = self._object_controller.main_occupancy_grid.is_free(goal[0], goal[1])
        # obstacles = np.array(self._object_controller.static_obstacle_positions, dtype = float).reshape(-1, 2)
        # return bool(np.all(np.linalg.norm(obstacles - start, axis = 1) > clearence) and np.all(np.linalg.norm(obstacles - goal, axis = 1) > clearence))
        return start_clear and goal_clear

    def set_start_goal_position(self, max_attempts: int = 10000) -> bool:
        for _ in range(max_attempts):
            start = np.random.uniform(-(self.arena_size - 1) / 2, (self.arena_size - 1) / 2, size=(2,))
            direction = np.random.uniform(-1.0, 1.0, size=(2,))
            goal = self.stage_settings[self.current_stage_name].goal_distance * (direction / np.linalg.norm(direction)) + start
            if np.any(np.abs(goal) > ((self.arena_size - 1) / 2)):
                continue
            if self.is_clear_of_static_obstacles(start, goal):
                self.start_position, self.goal_position = start, goal
                return True
        return False

    def apply_start_goal(self) -> None:
        if not self._object_controller.place_markers_and_robot(self.start_position, self.goal_position):
            self.get_logger().warn("Robot or start / goal markers not moved")
        self.publish_start_goal_markers()

    def fill_start_goal_response(self, response) -> None:
        response.initial_pos_x = float(self.start_position[0])
        response.initial_pos_y = float(self.start_position[1])
        response.goal_pos_x = float(self.goal_position[0])
        response.goal_pos_y = float(self.goal_position[1])
        response.goal_tolerance = self.stage_settings[self.current_stage_name].goal_tolerance

    def advance_inference_trial(self) -> bool:
        if self._inference_trial >= self.trials_per_stage:
            self._inference_trial = 0
            self._inference_stage_index += 1
        if self._inference_stage_index >= len(self.stage_names):
            return False
        if self._inference_trial == 0:
            if self.seed is not None:
                np.random.seed(self.seed + self._inference_stage_index)
            self.change_stage(self.stage_names[self._inference_stage_index])
        elif self.reroll_obstacles_each_trial:
            self.change_stage(self.stage_names[self._inference_stage_index])
        self._inference_trial += 1
        return True

    def handle_quick_reset_request(self, request, response):
        self.get_logger().info("handle quick reset simulation")
        self.__set_pose_controller_event.clear()
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
        response.stage = self.current_stage_name
        response.success = True
        response.message = f"\n| Current Stage Level: {self.current_stage_name}\n"
        response.message += "| New Stage: Yes\n" if request.change_stage != 0 else "| New Stage: No\n"
        response.message += f"| Current Start: [{self.start_position[0]}; {self.start_position[1]}]\n"
        response.message += f"| Current Goal: [{self.goal_position[0]}; {self.goal_position[1]}]\n"
        return response

    def handle_inference_scenario_request(self, reqrebpuest, response):
        self.get_logger().info("handle inference scenario")
        if not self.advance_inference_trial():
            response.done_evaluation = True
            response.success = True
            response.message = "\n| Inference evaluation finished\n"
            return response
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