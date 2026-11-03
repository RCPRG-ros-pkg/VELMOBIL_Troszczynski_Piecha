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

        self.grid_origin = map_origin / voxel_size
        self.grid = np.ones((int(self.map_shape[0] // self.voxel_size), int(self.map_shape[1] // self.voxel_size)), dtype=np.uint8) * FREE_SPACE
        self.inflated_grid = np.ones((int(self.map_shape[0] // self.voxel_size), int(self.map_shape[1] // self.voxel_size)), dtype=np.uint8) * FREE_SPACE
        self.obstacles : list[Obstacle] = []

    def clear(self):
        self.grid = np.ones((int(self.map_shape[0] // self.voxel_size), int(self.map_shape[1] // self.voxel_size)), dtype=np.uint8) * FREE_SPACE
        self.inflated_grid = np.ones((int(self.map_shape[0] // self.voxel_size), int(self.map_shape[1] // self.voxel_size)), dtype=np.uint8) * FREE_SPACE
        self.obstacles : list[Obstacle] = []

    def cast_world_to_grid(self, x_world, y_world):
        return (math.floor(x_world / self.voxel_size - self.grid_origin[0]), math.floor(y_world / self.voxel_size - self.grid_origin[1]))
    def cast_grid_to_world(self, x_grid, y_grid):
        return ((x_grid + self.grid_origin[0]) * self.voxel_size + int(self.voxel_size/2), (y_grid + self.grid_origin[1]) * self.voxel_size + int(self.voxel_size/2))
    
    def index_grid(self, x : int, y : int) -> tuple[int]:
        """Pass standard index, receive numpy index"""
        grid_center_x = int ((self.map_shape[0] - self.map_origin[0]) // self.voxel_size)
        grid_center_y = int ((self.map_shape[1] - self.map_origin[1]) // self.voxel_size)
        return (grid_center_x - x, grid_center_y - y)

    def reverse_index_grid(self, x : int, y : int) -> tuple[int]:
        """Pass numpy index, receive standard index"""
        grid_center_x = int ((self.map_shape[0] - self.map_origin[0]) // self.voxel_size)
        grid_center_y = int ((self.map_shape[1] - self.map_origin[1]) // self.voxel_size)
        return (grid_center_x -x, grid_center_y - y) ## no mistake - its the same



    def place_obstacle(self, obstacle : Obstacle):
        self.obstacles.append(obstacle)
        left_upper_corner = (obstacle.center_x + obstacle.size_x/2, obstacle.center_y + obstacle.size_y/2)
        right_lower_corner = (obstacle.center_x - obstacle.size_x/2, obstacle.center_y - obstacle.size_y/2)
        for x in range(int(left_upper_corner[0]), int(right_lower_corner[0])):
            for y in range(int(left_upper_corner[1]), int(right_lower_corner[1])):
                self.grid[self.index_grid(x,y)] = LETHAL_SPACE

    def remove_obstacle(self, obstacle_name: str):
        self.obstacles = [o for o in self.obstacles if o.name != obstacle_name]

    def inflate(self, inflation_range):
        for x in range(self.grid.shape[0]):
            for y in range(self.grid.shape[1]):
                if self.grid[x,y] != FREE_SPACE:
                    try:
                        self.inflated_grid[x-inflation_range : x+inflation_range , y-inflation_range : y+inflation_range] = LETHAL_SPACE
                    except:
                        pass # for now there is no bounds checking

    def is_free(self, x_world : float, y_world : float) -> bool:
        return True if self.inflated_grid[self.index_grid(*self.cast_world_to_grid(x_world, y_world))] == FREE_SPACE else False


    
        

    