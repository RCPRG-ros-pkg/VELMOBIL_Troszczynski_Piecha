import os
import numpy as np
import subprocess
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Pose


"""
Simulation Manager

+ Uses ObstacleController as a lower-level component responsible for spawning
  and configuring simulation objects.
+ Uses TimeController as a lower-level component responsible for controlling
  the real-time factor or executing multiple simulation steps during training.
"""


class ObstacleController:
    def __init__(self, static_obstacles_num: int, dynamic_obstacles_num: int, min_obstacle_speed: float, max_obstacle_speed: float):
        self._static_rect_obstacle_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'static_obstacles', 'static_obstacle_rect.sdf')
        self._dynamic_rect_obstacle_path = os.path.join(get_package_share_directory('velmobil_agent'), 'obstacles', 'dynamic_obstacles', 'dynamic_obstacle_rect.sdf')
        self._static_obstacles_num = static_obstacles_num
        self._dynamic_obstacles_num = dynamic_obstacles_num
        self._min_obstacle_speed = min_obstacle_speed
        self._max_obstacle_speed = max_obstacle_speed

    def spawn_static_obstacles(self, name: str, area_size, pose: Pose) -> None:
        positions = [(np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2)) for _ in range(self._static_obstacles_num)]
        includes = "\n".join(f"""<include>
                <uri>file://{self._static_rect_obstacle_path}</uri>
                <name>static_obstacle_{i}</name>
                <pose>{x} {y} 1.1 0 0 0</pose>
                </include>""" for i, (x, y) in enumerate(positions))
        sdf = f"""<?xml version="1.0"?>
                    <sdf version="1.9">
                    <model name="static_obstacles">
                        <static>true</static>
                        {includes}
                    </model>
                </sdf>"""
        pose = Pose()
        command = ['ros2', 'run', 'ros_gz_sim', 'create', 
                '-name', name, '-string', sdf, '-x', str(pose.position.x), '-y', str(pose.position.y), '-z', str(pose.position.z)]
        subprocess.run(command, capture_output=True, text=True, timeout=30.0)

    def set_velocities_for_dynamic_obstacles(self) -> list:
        pass



class TimeController:
    def __init__(self, rtf: float, steps: int, pause: bool):
        self._rtf = rtf
        self._steps = steps
        self._pause = pause

    def set_real_time_factor(self, world_name: str) -> None:
        command = ['gz', 'service', '-s', f'/world/{world_name}/set_physics',
            '--reqtype', 'gz.msgs.Physics', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', f'real_time_factor: {self._rtf}']
        subprocess.run(command, capture_output=True, text=True, timeout=30.0)

    def step_simulation(self, world_name: str) -> None:
        req = f'pause: {str(self._pause).lower()}, multi_step: {self._steps}'
        command = ['gz', 'service', '-s', f'/world/{world_name}/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', req]
        subprocess.run(command, capture_output=True, text=True, timeout=30.0)



class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager')
        self.simulation_timer = self.create_timer(1.0, self.manage_simulation)
        self._spawned = False
        self._world_name = 'empty'

        #   Parametrami area_size i static_obstacles_num można sterować zagęszczeniem przeszkód statycznych. 
        #   Zakres, w którym następuje spawn każdej przeszkody jest liczony od -area_size/2 do area_size/2 w obu osiach x i y.
        self.declare_parameter('area_size', 20)
        self.declare_parameter('static_obstacles_num', 15)
        self.declare_parameter('dynamic_obstacles_num', 5)
        self.declare_parameter('min_obstacle_speed', 0.5)
        self.declare_parameter('max_obstacle_speed', 2.0)

        self._area_size = self.get_parameter('area_size').value
        self._obstacle_controller = ObstacleController(static_obstacles_num=self.get_parameter('static_obstacles_num').value, 
                                                      dynamic_obstacles_num=self.get_parameter('dynamic_obstacles_num').value, 
                                                      min_obstacle_speed=self.get_parameter('min_obstacle_speed').value, 
                                                      max_obstacle_speed=self.get_parameter('max_obstacle_speed').value)

        #   Jeżeli if_rtf = True - wtedy setowany jest współczynnik RTF symulacji
        #   Jeżeli if_rtf = False - wybrano opcje multi - step symulacji
        self.declare_parameter('if_rtf', True)
        self._if_rtf = self.get_parameter('if_rtf').value

        #   rtf - real time factor
        self.declare_parameter('rtf', 1.0)
        self.declare_parameter('steps', 1)
        self.declare_parameter('pause', False)
        self._time_controller = TimeController(rtf=self.get_parameter('rtf').value, 
                                               steps=self.get_parameter('steps').value, 
                                               pause=self.get_parameter('pause').value)


    def manage_simulation(self):
        self.simulation_timer.cancel()
        self.spawn_obstacles()
        self.manage_time()

    def spawn_obstacles(self):
        if self._spawned:
            return
        self._spawned = True
        self._obstacle_controller.spawn_static_obstacles("static_obstacles", self._area_size, Pose())

    def manage_time(self):
        if self._if_rtf:
            self._time_controller.set_real_time_factor(self._world_name)
        else:
            self._time_controller.step_simulation(self._world_name)


