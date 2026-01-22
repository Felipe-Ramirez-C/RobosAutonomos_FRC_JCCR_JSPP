import os
import random
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    # 1. Randomization Logic
    points = [
        (1.0, 1.0), (13.0, 5.0), (-2.0, 5.0), 
        (-2.0, -5.0), (-10.0, 1.5), 
        (20.0, -3.0), (25.0, -3.0), (-8.0, 1.5)
    ]
    robot_point, person_point = random.sample(points, k=2)
    robot_yaw = random.uniform(0, 6.28)
    person_yaw = random.uniform(0, 6.28)

    # 2. Path Setup
    home = str(Path.home())
    setup_path = os.path.join(home, 'clearpath') 
    setup_path_plus_slash = os.path.join(setup_path, '')
    map_path = os.path.join(setup_path, 'office_map.yaml')
    namespace = 'a200_0000'

    # --- GROUP A: START IMMEDIATELY (Gazebo) ---
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('clearpath_gz'), 'launch', 'simulation.launch.py')
        ]),
        launch_arguments={
            'world': 'office',
            'setup_path': setup_path_plus_slash,
            'use_sim_time': 'true',
            'x': str(robot_point[0]),
            'y': str(robot_point[1]),
            'yaw': str(robot_yaw)
        }.items()
    )

    # --- GROUP B: START AFTER 15s (RViz) ---
    visualization = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('clearpath_viz'), 'launch', 'view_navigation.launch.py')
                ]),
                launch_arguments={'namespace': namespace}.items()
            )
        ]
    )

    # --- GROUP C: START AFTER 25s (Nav2 & Localization) ---
    # Increased to 25s to prevent the "odom" transform errors seen in your log
    nav_stack = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('clearpath_nav2_demos'), 'launch', 'localization.launch.py')
                ]),
                launch_arguments={'setup_path': setup_path_plus_slash, 'use_sim_time': 'true', 'map': map_path}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('clearpath_nav2_demos'), 'launch', 'nav2.launch.py')
                ]),
                launch_arguments={'setup_path': setup_path_plus_slash, 'use_sim_time': 'true', 'map': map_path, 'namespace': namespace}.items()
            )
        ]
    )

    # --- GROUP D: START AFTER 30s (YOLO & Person) ---
    yolo_and_person = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='vision_robot_yolo', 
                executable='yolov8_robot', 
                name='yolov8_robot', 
                parameters=[{'use_sim_time': True}]
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', 'office',
                    '-file', 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing Person',
                    '-name', 'random_person',
                    '-x', str(person_point[0]), '-y', str(person_point[1]), '-z', '0.1', '-Y', str(person_yaw)
                ],
                output='screen'
            )
        ]
    )

    # --- GROUP E: START AFTER 35s (Wall Tracking) ---
    wall_track = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='wall_tracking',
                executable='wall_tracking_robot',
                name='wall_tracking_node',
                namespace=namespace,
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        simulation,
        visualization,
        nav_stack,
        yolo_and_person,
        wall_track
    ])
