import numpy as np


class ObstacleController:
    def __init__(self, static_obstacle_num: int, dynamic_obstacles_num: int, min_obstacle_speed: float, max_obstacle_speed: float):
        self.static_obstacle_num = static_obstacle_num
        self.dynamic_obstacles_num = dynamic_obstacles_num
        self.min_obstacle_speed = min_obstacle_speed
        self.max_obstacle_speed = max_obstacle_speed

    def generate_static_obstacle_positions(self, area_size: int = 10) -> list:
        return [(np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2)) for _ in range(self.static_obstacle_num)]

    def generate_dynamic_obstacle_positions_and_speeds(self, area_size: int = 10) -> list:
        return [(np.random.randint(-area_size / 2, area_size / 2), np.random.randint(-area_size / 2, area_size / 2), np.random.uniform(self.min_obstacle_speed, self.max_obstacle_speed)) for _ in range(self.dynamic_obstacles_num)]

    def set_velocities_for_dynamic_obstacles(self) -> list:
        pass