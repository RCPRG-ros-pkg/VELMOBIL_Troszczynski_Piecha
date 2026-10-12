#!/usr/bin/env python3
import rclpy
import subprocess
from rclpy.node import Node


class TimeController(Node):
    def __init__(self):
        super().__init__('time_controller')
        self._world_name = 'velmobil_arena'

        self.declare_parameter('rtf', 1.0)
        self.declare_parameter('steps', 1)
        self.declare_parameter('pause', False)

        self._rtf = self.get_parameter('rtf').value
        self._steps = self.get_parameter('steps').value
        self._pause = self.get_parameter('pause').value

    def set_real_time_factor(self) -> None:
        command = ['gz', 'service', '-s', f'/world/{self._world_name}/set_physics',
            '--reqtype', 'gz.msgs.Physics', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', f'real_time_factor: {self._rtf}']
        subprocess.run(command, capture_output=True, text=True, timeout=30.0)

    def step_simulation(self) -> None:
        req = f'pause: {str(self._pause).lower()}, multi_step: {self._steps}'
        command = ['gz', 'service', '-s', f'/world/{self._world_name}/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', req]
        subprocess.run(command, capture_output=True, text=True, timeout=30.0)

    def manage_time(self):
        self.set_real_time_factor()
        self.step_simulation()



def main():
    rclpy.init()
    time_controller = TimeController()
    try:
        time_controller.manage_time()
    except KeyboardInterrupt:
        pass
    finally:
        time_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()