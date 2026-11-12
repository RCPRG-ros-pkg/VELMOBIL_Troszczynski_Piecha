#!/usr/bin/env python3
from dataclasses import dataclass, field


@dataclass
class Obstacle:
    name: str
    center_x: float
    center_y: float
    size_x: float
    size_y: float