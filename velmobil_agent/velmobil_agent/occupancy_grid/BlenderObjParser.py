#!/usr/bin/env python3
import os
from ament_index_python import get_package_share_directory
from velmobil_agent.occupancy_grid.Obstacle import Obstacle


def get_path_to_obj_map(map_name) -> str:
    map_path = os.path.join(get_package_share_directory("velmobil_simulation"), "worlds", "models", map_name, f"{map_name}.obj")
    return map_path


def blender_obj_parser(filepath : str) -> list[Obstacle]:
    obstacle_list : list[Obstacle] = []
    name = None
    xs, ys = [], []

    def add_obstacle():
        nonlocal name, xs, ys
        if name is None or not xs:
            return
        obstacle_list.append(Obstacle(
            name=name,
            center_x=(min(xs) + max(xs)) / 2,
            center_y=(min(ys) + max(ys)) / 2,
            size_x=max(xs) - min(xs),
            size_y=max(ys) - min(ys)
            )
        )
        xs, ys = [], []

    with open(filepath, 'r') as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'o':
                add_obstacle()
                name = parts[1]
            if parts[0] == 'v':
                x, y, z = map(float, parts[1:4])
                xs.append(x)
                ys.append(y)
    add_obstacle()
    return obstacle_list