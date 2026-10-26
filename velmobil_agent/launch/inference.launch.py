from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
import os


def generate_launch_description():
    pkg = get_package_share_directory('velmobil_agent')
    params_file = os.path.join(pkg, 'config', 'inference_params.yaml')

    manager = Node(
        package='velmobil_agent',
        executable='manage_simulation',
        name='simulation_manager',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
        emulate_tty=True
    )

    agent = Node(
        package='velmobil_agent',
        executable='agent_inference_drl',
        parameters=[{'use_sim_time': True}],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        manager,
        RegisterEventHandler(OnProcessStart(
            target_action=manager,
            on_start=[TimerAction(period=3.0, actions=[agent])],
        )),
    ])