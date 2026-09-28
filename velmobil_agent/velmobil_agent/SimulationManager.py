import subprocess
from rclpy.node import Node
from geometry_msgs.msg import Pose


def load_object(path: str):
    with open(path, 'r') as f:
        return f.read()


class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager')
        self.get_logger().info('SimulationManager initialized')
        self.spawn_timer = self.create_timer(1.0, self.spawn_object)

    def spawn_object(self):
        self.spawn_timer.cancel()
        name = 'test'

        sdf = load_object("install/velmobil_agent/share/velmobil_agent/obstacles/static_obstacles/static_obstacle_rect.sdf")

        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 1.0
        pose.position.z = 1.1

        self.spawn(name=name, sdf=sdf, pose=pose)

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
