#!/usr/bin/env python3
from velmobil_agent.occupancy_grid.Obstacle import Obstacle
import math
import numpy as np


FREE_SPACE = 0
LETHAL_SPACE = 255


class OccupancyGrid:
    def __init__(self, voxel_size : float, map_shape : np.ndarray, map_origin : np.ndarray):
        self.voxel_size = voxel_size
        self.map_shape = map_shape
        self.map_origin = map_origin
        self.n_x = int(round(map_shape[0] / voxel_size))
        self.n_y = int(round(map_shape[1] / voxel_size))
        self.clear()

    def clear(self):
        self.grid = np.ones((self.n_x, self.n_y), dtype=np.uint8) * FREE_SPACE
        self.inflated_grid = np.ones((self.n_x, self.n_y), dtype=np.uint8) * FREE_SPACE
        self.obstacles : list[Obstacle] = []

    def cast_world_to_grid(self, x_world : float, y_world : float) -> tuple[int, int]:
        return (math.floor((x_world - self.map_origin[0]) / self.voxel_size),
                math.floor((y_world - self.map_origin[1]) / self.voxel_size))

    def cast_grid_to_world(self, x_grid : int, y_grid : int) -> tuple[float, float]:
        return (self.map_origin[0] + (x_grid + 0.5) * self.voxel_size,
                self.map_origin[1] + (y_grid + 0.5) * self.voxel_size)

    def index_grid(self, x : int, y : int) -> tuple[int, int]:
        """Pass standard index, receive numpy index"""
        return (self.n_x - 1 - x, self.n_y - 1 - y)

    def reverse_index_grid(self, row : int, col : int) -> tuple[int, int]:
        """Pass numpy index, receive standard index"""
        return (self.n_x - 1 - row, self.n_y - 1 - col) ## no mistake - its the same

    def in_bounds(self, x : int, y : int) -> bool:
        """Check standard index"""
        return 0 <= x < self.n_x and 0 <= y < self.n_y

    def place_obstacle(self, obstacle : Obstacle):
        self.obstacles.append(obstacle)
        x_min, y_min = self.cast_world_to_grid(obstacle.center_x - obstacle.size_x / 2, obstacle.center_y - obstacle.size_y / 2)
        x_max, y_max = self.cast_world_to_grid(obstacle.center_x + obstacle.size_x / 2, obstacle.center_y + obstacle.size_y / 2)
        for x in range(x_min, x_max):
            for y in range(y_min, y_max):
                if self.in_bounds(x, y):
                    self.grid[self.index_grid(x, y)] = LETHAL_SPACE

    def remove_obstacle(self, obstacle_name: str):
        self.obstacles = [o for o in self.obstacles if o.name != obstacle_name]

    def inflate(self, inflation_range : int):
        self.inflated_grid = self.grid.copy()
        for row, col in np.argwhere(self.grid != FREE_SPACE):
            self.inflated_grid[max(row - inflation_range, 0) : row + inflation_range + 1,
                               max(col - inflation_range, 0) : col + inflation_range + 1] = LETHAL_SPACE

    def is_free(self, x_world : float, y_world : float) -> bool:
        """Points outside the map are treated as occupied"""
        x, y = self.cast_world_to_grid(x_world, y_world)
        return self.in_bounds(x, y) and self.inflated_grid[self.index_grid(x, y)] == FREE_SPACE
