#!/usr/bin/env python3
import os
import numpy as np
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from velmobil_interfaces.srv import TrainingPhase
from std_srvs.srv import Trigger
from gz.transport13 import Node as GZNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

"""
Simulation Manager
+ responsible for spawning and configuring simulation objects by low level component ObjectController.
"""


class ObjectController:
    def __init__(self, static_obstacles_num: int, dynamic_obstacles_num: int, min_obstacle_speed: float, max_obstacle_speed: float):
        self._world_name = 'velmobil_arena'
        self._static_rect_obstacle_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'static_obstacle_rect.sdf')
        self._goal_object_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'goal_object.sdf')
        self._dynamic_rect_obstacle_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'dynamic_obstacles', 'dynamic_obstacle_rect.sdf')
        self._static_obstacles_num = static_obstacles_num
        self._dynamic_obstacles_num = dynamic_obstacles_num
        self._min_obstacle_speed = min_obstacle_speed
        self._max_obstacle_speed = max_obstacle_speed
        self._gz_node = GZNode()

    def spawn_static_obstacles(self, area_size) -> None:
        positions = [(np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2)) for _ in range(self._static_obstacles_num)]
        procs = []
        for i, (x, y) in enumerate(positions):
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

    def set_velocity_for_dynamic_obstacles(self, publisher) -> None:
        msg = Twist()
        msg.linear.x = np.random.uniform(self._min_obstacle_speed, self._max_obstacle_speed)
        publisher.publish(msg)



class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager')
        self.get_logger().info("[SimulationManager] initialization")
        self.robot_initial_position = [0.0, 0.0]
        self.goal_position = [5.0, 0.0]



        self._dynamic_obstacles_publisher = self.create_publisher(Twist, '/model/dynamic_obstacle_rectangle/cmd_vel', 10)
        self._velmobil_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._reset_service = self.create_service(Trigger, '/reset_simulation', self.handle_reset_request)



        self.declare_parameter('area_size', 12)
        self.declare_parameter('static_obstacles_num', 15)
        self.declare_parameter('dynamic_obstacles_num', 0)
        self.declare_parameter('min_obstacle_speed', 0.5)
        self.declare_parameter('max_obstacle_speed', 2.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('goal_distance', 5.0)

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_distance = self.get_parameter('goal_distance').value

        self._area_size = self.get_parameter('area_size').value
        self._object_controller = ObjectController(static_obstacles_num=self.get_parameter('static_obstacles_num').value, 
                                                   dynamic_obstacles_num=self.get_parameter('dynamic_obstacles_num').value, 
                                                   min_obstacle_speed=self.get_parameter('min_obstacle_speed').value, 
                                                   max_obstacle_speed=self.get_parameter('max_obstacle_speed').value)
        self._object_controller.spawn_goal_object(self.goal_position)

    def handle_reset_request(self, request, response):
        self.get_logger().info("handle reset simulation")
        robot_teleportation_result = self._object_controller.teleport_robot(self.robot_initial_position[0], self.robot_initial_position[1])
        move_goal_result = self._object_controller.move_goal_object(self.goal_position)
        response.success = robot_teleportation_result and move_goal_result
        response.message = "simulation reset successfully" if response.success else f'reset failed (robot: {robot_teleportation_result}, goal: {move_goal_result})'
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