import os
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    
    ld = LaunchDescription()
    
    start_training = Node(
        package='rl_tools',
        executable='training_node',
        output='screen'
        )
    
    ld.add_action(start_training)
    
    return ld
    
    