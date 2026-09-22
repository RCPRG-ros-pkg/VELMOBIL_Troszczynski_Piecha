import os
import xacro
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


"""
QUICK GUIDE:
floating - (True/False) - floating_controller 
roll_yaw - (True/False) - roll_yaw_controller
rviz - (True/False) - Rviz ON / Rviz OFF
world - Full path to the SDF world file to load
"""

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
    models_path = PathJoinSubstitution([
        velmobil_simulation,
        'worlds',
        'models'
    ])

    robot_controller_floating = PathJoinSubstitution([
        velmobil_description,
        'control_config',
        'floating_velmobil.yaml'
    ])
    robot_controller_roll_yaw = PathJoinSubstitution([
        velmobil_description,
        'control_config',
        'roll_yaw_velmobil.yaml'
    ])
    velmobil_urdf = PathJoinSubstitution([
        velmobil_description,
        'urdf',
        'velmobil.urdf.xacro'
    ])



    # ARGUMENTS
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    rviz = LaunchConfiguration('rviz', default=True)
    world = LaunchConfiguration('world')
    floating = LaunchConfiguration('floating', default=False)
    roll_yaw = LaunchConfiguration('roll_yaw', default=True)
    realsense = LaunchConfiguration('realsense', default=False)


    # XACRO COMMAND
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        velmobil_urdf,
        ' ',
        'floating:=', floating,
        ' ',
        'roll_yaw:=', roll_yaw,
        ' ',
        'realsense:=', realsense
    ])

    set_ign_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=PathJoinSubstitution([models_path, os.pathsep, os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    )
    

    
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
                   '-name', 'velmobil', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    floating_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['floating_controller', '--param-file', robot_controller_floating],
        condition=IfCondition(floating)
    )
    roll_yaw_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['roll_yaw_controller', '--param-file', robot_controller_roll_yaw],
        condition=IfCondition(roll_yaw)
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/right/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/left/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/front_depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/front_depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/front_depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/back_depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/back_depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/back_depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
        ],
        parameters=[{'use_sim_time': True}],
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

    lidar_merger = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laser_merger',
        parameters=[{
            'destination_frame': 'base_link',
            'scan_destination_topic': '/lidar_fusion',
            'laserscan_topics': '/left/scan /right/scan',
            'angle_increment': 0.0174533,
            'range_min': 0.06,
            'range_max': 8.0
        }],
        output='screen'
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
        DeclareLaunchArgument(
            'floating',
            default_value='false',
            description='If true, floating motion model will apply'
        ),
        DeclareLaunchArgument(
            'roll_yaw',
            default_value='true',
            description='If true, roll_yaw motion model will apply'
        ),
        DeclareLaunchArgument(
            'realsense',
            default_value='false',
            description='If true, realsense cameras are enabled in gazebo'
        ),

        set_ign_path,
        
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
                on_exit=[roll_yaw_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[floating_controller_spawner],
            )
        ),
        
        bridge,
        robot_state_publisher,
        gz_spawn_entity,
        rviz_node,
        lidar_merger
    ])