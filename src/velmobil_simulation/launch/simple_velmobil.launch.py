import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # PACKAGES
    velmobil_description = FindPackageShare('velmobil_description')
    velmobil_simulation = FindPackageShare('velmobil_simulation')
    ros_gz_sim = FindPackageShare('ros_gz_sim')


    # CONFIG FILES
    rviz_config_path = PathJoinSubstitution([
        velmobil_simulation,
        'rviz',
        'basic.rviz'
    ])
    bridge_config = PathJoinSubstitution([
        velmobil_simulation,
        'gz_bridge',
        'bridge_config.yaml'
    ])
    robot_controllers = PathJoinSubstitution([
        velmobil_description,
        'control',
        'simple_velmobil.yaml'
    ])
    simple_velmobil_urdf = PathJoinSubstitution([
        velmobil_description,
        'urdf',
        'simple_velmobil.urdf.xacro'
    ])



    # ARGUMENTS
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    rviz = LaunchConfiguration('rviz', default=True)
    world = LaunchConfiguration('world')


    # XACRO COMMAND
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        simple_velmobil_urdf,
    ])
    


    # NODES
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'simple_velmobil', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--param-file', robot_controllers]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[bridge_config],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(rviz) 
    )








    # RETURN
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                velmobil_simulation, 
                'worlds', 
                'basic.sdf'
            ]),
            description='Full path to the SDF world file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='If true, rviz will launch'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([ros_gz_sim,
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [world, ' -r -v 1'])]),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[mecanum_drive_controller_spawner],
            )
        ),
        
        bridge,
        robot_state_publisher,
        gz_spawn_entity,
        rviz_node,
    ])