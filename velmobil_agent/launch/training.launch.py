import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('velmobil_agent')
    params_file = os.path.join(pkg, 'training_config', 'training_params.yaml')

    manager = Node(
        package='velmobil_agent',
        executable='manage_simulation',
        name='simulation_manager',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
    )

    agent = Node(
        package='velmobil_agent',
        executable='agent_training_drl',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        manager,
        RegisterEventHandler(OnProcessStart(
            target_action=manager,
            on_start=[TimerAction(period=12.0, actions=[agent])],
        )),
    ])