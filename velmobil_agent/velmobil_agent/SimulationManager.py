#!/usr/bin/env python3
import os
import numpy as np
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from velmobil_interfaces.srv import StageModification
from std_srvs.srv import Trigger
from dataclasses import dataclass, fields
from gz.transport13 import Node as GZNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

"""
Simulation Manager
+ responsible for spawning and configuring simulation objects by low level component ObjectController.
"""

@dataclass(frozen=True)
class StageSettings:
    static_obstacles_num: int
    dynamic_obstacles_num: int
    min_obstacle_speed: float
    max_obstacle_speed: float
    goal_distance: float
    goal_tolerance: float


class ObjectController:
    def __init__(self, static_obstacles_num: int, dynamic_obstacles_num: int, min_obstacle_speed: float, max_obstacle_speed: float):
        self._world_name = 'velmobil_arena'
        self._static_rect_obstacle_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'static_obstacle_rect.sdf')
        self._goal_object_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'goal_object.sdf')
        self._start_object_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'start_object.sdf')
        self._dynamic_rect_obstacle_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'dynamic_obstacles', 'dynamic_obstacle_rect.sdf')
        self._static_obstacles_num = static_obstacles_num
        self._dynamic_obstacles_num = dynamic_obstacles_num
        self._min_obstacle_speed = min_obstacle_speed
        self._max_obstacle_speed = max_obstacle_speed
        self._gz_node = GZNode()

        self.static_obstacle_positions = []

    def set_stage_settings(self, stage_settings: StageSettings):
        self._static_obstacles_num = stage_settings.static_obstacles_num
        self._dynamic_obstacles_num = stage_settings.dynamic_obstacles_num
        self._min_obstacle_speed = stage_settings.min_obstacle_speed
        self._max_obstacle_speed = stage_settings.max_obstacle_speed

        self.static_obstacle_positions = []

    def set_stage_settings(self, stage_settings: StageSettings):
        self._static_obstacles_num = stage_settings.static_obstacles_num
        self._dynamic_obstacles_num = stage_settings.dynamic_obstacles_num
        self._min_obstacle_speed = stage_settings.min_obstacle_speed
        self._max_obstacle_speed = stage_settings.max_obstacle_speed

    def spawn_static_obstacles(self, area_size) -> None:
        picked_positions = 0
        while picked_positions < self._static_obstacles_num:
            position = [(np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2))]
            if position not in self.static_obstacle_positions:
                self.static_obstacle_positions.append(position)
                picked_positions += 1
        procs = []
        for i, (x, y) in enumerate(self.static_obstacle_positions):
            command = ['ros2', 'run', 'ros_gz_sim', 'create','-file', self._static_rect_obstacle_path, '-name', f'static_obstacle_{i}', '-x', str(x), '-y', str(y), '-z', '1.1']
            procs.append((i, subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)))
        for i, p in procs:
            try:
                _, err = p.communicate(timeout=90.0)
            except subprocess.TimeoutExpired:
                p.kill()
                print(f"Spawn failed for static_obstacle_{i}: {err.strip()}")

    def spawn_dynamic_obstacles(self, area_size) -> None:
        positions_orientations = [(-np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2), np.random.uniform(-np.pi, np.pi)) for _ in range(self._dynamic_obstacles_num)]
        procs = []
        for i, (x, y, yaw) in enumerate(positions_orientations):
            command = ['ros2', 'run', 'ros_gz_sim', 'create','-file', self._dynamic_rect_obstacle_path, '-name', f'dynamic_obstacle_{i}', '-x', str(x), '-y', str(y), '-z', '1.1', '-Y', str(yaw)]
            procs.append((i, subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)))
        for i, p in procs:
            try:
                _, err = p.communicate(timeout=90.0)
            except subprocess.TimeoutExpired:
                p.kill()
                print(f"Spawn failed for dynamic_obstacle_{i}: {err.strip()}")

    def remove(self, name: str):
        req = f'name: "{name}" type: MODEL'
        command = ['gz', 'service', '-s', f'/world/{self._world_name}/remove',
                '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000', '--req', req]
        return name, subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def remove_static_obstacles(self) -> bool:
        procs = [self.remove(f'static_obstacle_{i}') for i in range(self._static_obstacles_num)]
        for name, p in procs:
            try:
                _, err = p.communicate(timeout=30.0)
                self.static_obstacle_positions.clear()
            except subprocess.TimeoutExpired:
                p.kill()
                print(f'Remove timeout for {name}: {err.strip()}', flush=True) 

    def remove_dynamic_obstacles(self) -> bool:
        procs = [self.remove(f'dynamic_obstacle_{i}') for i in range(self._dynamic_obstacles_num)]
        for name, p in procs:
            try:
                _, err = p.communicate(timeout=30.0)
            except subprocess.TimeoutExpired:
                p.kill()
                print(f'Remove timeout for {name}: {err.strip()}', flush=True) 

    def spawn_goal_object(self, goal_coordinates: list) -> None:
        command = ['ros2', 'run', 'ros_gz_sim', 'create','-file', self._goal_object_path, '-name', 'goal_object', '-x', str(goal_coordinates[0]), '-y', str(goal_coordinates[1]), '-z', '0.0']
        subprocess.run(command, capture_output=True, text=True, timeout=90.0)

    def set_entity_pose(self, name: str, x: float, y: float, z: float = 0.0, timeout_ms: int = 2000) -> bool:
        req = Pose()
        req.name = name
        req.position.x = float(x)
        req.position.y = float(y)
        req.position.z = float(z)
        req.orientation.w = 1.0
        result, response = self._gz_node.request(f'/world/{self._world_name}/set_pose', req, Pose, Boolean, timeout_ms)
        if not result:
            print(f'set_pose request failed/timed out for {name}', flush=True)
            return False
        return response.data

    def move_goal_object(self, goal) -> bool:
        return self.set_entity_pose('goal_object', goal[0], goal[1])

    def teleport_robot(self, x: float, y: float) -> bool:
        return self.set_entity_pose('velmobil', x, y, 0.1)

    def spawn_start_object(self, start_coordinates: list) -> None:
        command = ['ros2', 'run', 'ros_gz_sim', 'create','-file', self._start_object_path, '-name', 'start_object', '-x', str(start_coordinates[0]), '-y', str(start_coordinates[1]), '-z', '0.0']
        subprocess.run(command, capture_output=True, text=True, timeout=90.0)
    def move_start_object(self, start) -> None:
        try:
            req = (f'name: "start_object", position: {{x: {start[0]}, y: {start[1]}, z: 0.0}}, '
                f'orientation: {{w: 1.0}}')
            cmd = ['gz', 'service', '-s', f'/world/{self._world_name}/set_pose',
                '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000', '--req', req]
            subprocess.run(cmd, capture_output=True, text=True, timeout=30.0)
        except subprocess.CalledProcessError as e:
            print(f'move_goal_object failed : {e.output.decode()}')

    def set_velocity_for_dynamic_obstacles(self, publisher) -> None:
        msg = Twist()
        msg.linear.x = np.random.uniform(self._min_obstacle_speed, self._max_obstacle_speed)
        publisher.publish(msg)

    def teleport_robot(self, x: float, y: float) -> None:
        req = (f'name: "velmobil", '
            f'position: {{x: {x}, y: {y}, z: 0.1}}, '
            f'orientation: {{x: 0, y: 0, z: 0.0, w: 1.0}}')
        command = ['gz', 'service', '-s', f'/world/{self._world_name}/set_pose',
                '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000', '--req', req]
        subprocess.run(command, capture_output=True, text=True, timeout=30.0)





class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager', automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("[SimulationManager] initialization")
        self._dynamic_obstacles_publisher = self.create_publisher(Twist, '/model/dynamic_obstacle_rectangle/cmd_vel', 10)
        self._quick_reset_service = self.create_service(Trigger, '/quick_reset_simulation', self.handle_quick_reset_request)
        self._hard_reset_service = self.create_service(StageModification, '/hard_reset_simulation', self.handle_hard_reset_request)

        self.start_position = [0.0, 0.0]
        self.goal_position = [0.0, 0.0]

        self.arena_size = self.get_parameter("arena_size").value
        stage_names = self.get_parameter("stage_names").value
        self.stage_settings = {}
        for name in stage_names:
            raw = self.get_parameters_by_prefix(f"stages.{name}")
            self.stage_settings[name] = StageSettings(**{k: p.value for k, p in raw.items()})
        self.get_logger().info(f"[SimulationManager] Acquired {len(self.stage_settings)} Stage Settings")
        self.current_stage_name = "s1"

        self._object_controller = ObjectController(static_obstacles_num=self.stage_settings[self.current_stage_name].static_obstacles_num, 
                                                   dynamic_obstacles_num=self.stage_settings[self.current_stage_name].dynamic_obstacles_num, 
                                                   min_obstacle_speed=self.stage_settings[self.current_stage_name].min_obstacle_speed, 
                                                   max_obstacle_speed=self.stage_settings[self.current_stage_name].max_obstacle_speed)
        self._object_controller.spawn_goal_object([0.0, 0.0])
        self._object_controller.spawn_start_object([0.0, 0.0])
        self.get_logger().info(f"[SimulationManager] Initialized Object Controller for {self.current_stage_name}")


    def set_start_goal_position(self, obstacles : list[list[float]] = []):
        obstacles = np.array(obstacles, dtype=float).reshape(-1, 2)
        while True:
            start_position = np.random.uniform(-((self.arena_size-1)/2), ((self.arena_size-1)/2), size=(2,))
            goal_position_vec = np.random.uniform(-1.0, 1.0, size=(2,))
            goal_position = self.stage_settings[self.current_stage_name].goal_distance * (goal_position_vec / np.linalg.norm(goal_position_vec)) + start_position

            if np.all(np.linalg.norm(obstacles - start_position, axis=1) > 1.5) & np.all(np.linalg.norm(obstacles - goal_position, axis=1) > 1.5):
                self.goal_position = goal_position
                self.start_position = start_position
                break



    def handle_quick_reset_request(self, request, response):
        self.get_logger().info("handle quick reset simulation")
        self._object_controller.teleport_robot(self.start_position)
        self._object_controller.move_goal_object(self.goal_position)
        self._object_controller.move_start_object(self.start_position)
        response.success = True
        response.message = "\n| Simulation quick reset executed successfully\n"
        return response

    def handle_hard_reset_request(self, request, response):
        self.get_logger().info("handle hard reset simulation")
        if request.change_stage != 0:
            self._object_controller.remove_static_obstacles()
            self._object_controller.remove_dynamic_obstacles()

            stage_index = int(self.current_stage_name[1:])
            match request.change_stage:
                case request.STAGE_UP: #
                    stage_index += 1
                case request.STAGE_DOWN:
                    stage_index -= 1
            self.current_stage_name = f"s{stage_index}"

            self._object_controller.set_stage_settings(self.stage_settings[self.current_stage_name])

            self._object_controller.spawn_dynamic_obstacles(self.arena_size)
            self._object_controller.spawn_static_obstacles(self.arena_size)

            self.set_start_goal_position(self._object_controller.static_obstacle_positions)

        elif request.change_stage == 0 & request.change_start_goal:
            self.set_start_goal_position(self._object_controller.static_obstacle_positions)


        self._object_controller.teleport_robot(self.start_position)
        self._object_controller.move_goal_object(self.goal_position)
        self._object_controller.move_start_object(self.start_position)

        response.initial_pos_x = self.start_position[0]
        response.initial_pos_y = self.start_position[1]
        response.goal_pos_x = self.goal_position[0]
        response.goal_pos_x = self.goal_position[1]
        response.goal_tolerance = self.stage_settings[self.current_stage_name].goal_tolerance
        response.success = True
        response.message = f"\n| Current Stage Level: {self.current_stage_name}\n"
        response.message += f"| New Stage: Yes\n" if request.change_stage != 0 else "| New Stage: No\n"
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