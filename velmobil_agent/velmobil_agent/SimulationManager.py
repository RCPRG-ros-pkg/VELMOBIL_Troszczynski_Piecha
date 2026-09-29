import os
import numpy as np
import subprocess
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Pose



class ObstacleController:
    def __init__(self, static_obstacles_num: int, dynamic_obstacles_num: int, min_obstacle_speed: float, max_obstacle_speed: float):
        self._static_rect_obstacle_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'static_obstacle_rect.sdf')
        self._dynamic_rect_obstacle_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'dynamic_obstacles', 'dynamic_obstacle_rect.sdf')
        self.static_obstacles_num = static_obstacles_num
        self.dynamic_obstacles_num = dynamic_obstacles_num
        self.min_obstacle_speed = min_obstacle_speed
        self.max_obstacle_speed = max_obstacle_speed

    def create_static_obstacles(self, area_size: int) -> str:
        positions = [(np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2)) for _ in range(self.static_obstacles_num)]
        includes = "\n".join(f"""<include>
                <uri>file://{self._static_rect_obstacle_path}</uri>
                <name>static_obstacle_{i}</name>
                <pose>{x} {y} 1.1 0 0 0</pose>
                </include>""" for i, (x, y) in enumerate(positions))
        return f"""<?xml version="1.0"?>
                    <sdf version="1.9">
                    <model name="static_obstacles">
                        <static>true</static>
                        {includes}
                    </model>
                </sdf>"""

    def set_velocities_for_dynamic_obstacles(self) -> list:
        pass


class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager')
        self.spawn_timer = self.create_timer(1.0, self.spawn_many_objects)
        self._spawned = False

        self.declare_parameter('area_size', 12)
        self.declare_parameter('static_obstacles_num', 15)
        self.declare_parameter('dynamic_obstacles_num', 5)
        self.declare_parameter('min_obstacle_speed', 0.5)
        self.declare_parameter('max_obstacle_speed', 2.0)

        self._area_size = self.get_parameter('area_size').value
        self._obstacle_controller = ObstacleController(static_obstacles_num=self.get_parameter('static_obstacles_num').value, 
                                                      dynamic_obstacles_num=self.get_parameter('dynamic_obstacles_num').value, 
                                                      min_obstacle_speed=self.get_parameter('min_obstacle_speed').value, 
                                                      max_obstacle_speed=self.get_parameter('max_obstacle_speed').value)

    def spawn_many_objects(self):
        if self._spawned:
            return
        self._spawned = True
        self.spawn_timer.cancel()
        sdf = self._obstacle_controller.create_static_obstacles(area_size=self._area_size)
        self.spawn(name="static_obstacles", sdf=sdf, pose=Pose())
        

    def spawn(self, name: str, sdf: str, pose: Pose):
        command = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', name, '-string', sdf, '-x', str(pose.position.x), '-y', str(pose.position.y), '-z', str(pose.position.z)]
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=30.0)
            if result.returncode == 0:
                self.get_logger().info(f"Entity '{name}' spawned successfully.")
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")
