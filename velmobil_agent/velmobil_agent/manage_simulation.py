#!/usr/bin/env python3
import rclpy
from .SimulationManager import SimulationManager


def main():
    rclpy.init()
    simulation_manager = SimulationManager()
    try:
        simulation_manager.manage_simulation()
    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()