"""
File: multirotor_standalone.launch.py
Author: Brandon Sutherland, Jacob Moore
Created: February 3, 2025
Last Modified: March 25, 2025
Description: ROS2 launch file used to launch all the nodes for a Multirotor standalone simulator
"""

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that runs the bare minimum requirements fly a multirotor in a standalone simulator"""
    dynamics_param_file = os.path.join(get_package_share_directory('rosflight_sim'), 'params', 'multirotor_dynamics.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the nodes will use sim time or not"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Forwarded to common_nodes_standalone, which declares it but was never passed it from here --
    # so it could only ever take its default and the RC node could not be switched to vimfly from
    # the top-level launch.
    use_vimfly_arg = DeclareLaunchArgument(
        "use_vimfly",
        default_value="false",
        description="Use vimfly (keyboard RC) instead of a physical transmitter."
    )
    use_vimfly = LaunchConfiguration('use_vimfly')

    # Forwarded to standalone_sim, which owns mock_environment. Selects the obstacle world;
    # its default selects the built-in demo obstacles.
    environment_file_arg = DeclareLaunchArgument(
        "environment_file",
        default_value=os.path.join(get_package_share_directory("rosflight_sim"),
                                   "params", "environment_demo.yaml"),
        description="Obstacle-world YAML, e.g. params/forest_medium.yaml."
    )
    environment_file = LaunchConfiguration('environment_file')

    lidar_output_frame_arg = DeclareLaunchArgument(
        "lidar_output_frame",
        default_value="ned",
        description="Mock LiDAR output: 'ned' (world, truth pose) or 'sensor' (raw, needs "
                    "use_lidar_preprocessor:=true on roscopter_sim's sim.launch.py)."
    )
    lidar_output_frame = LaunchConfiguration('lidar_output_frame')

    ##########
    # Launch #
    ##########

    # Start simulator
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch/standalone_sim.launch.py",
            )
        ]),
        launch_arguments={
            'sim_aircraft_file': os.path.join("common_resource", "multirotor.dae"),
            'environment_file': environment_file,
            'lidar_output_frame': lidar_output_frame,
        }.items()
    )

    # Start common nodes
    common_nodes_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch", "common_nodes_standalone.launch.py"
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'dynamics_param_file': dynamics_param_file,
            'use_vimfly': use_vimfly,
        }.items()
    )

    # Start forces and moments
    mr_forces_moments_node = Node(
        package="rosflight_sim",
        executable="multirotor_forces_and_moments",
        name='multirotor_forces_and_moments',
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time}, dynamics_param_file
        ],
    )

    # Start dynamics node
    standalone_dynamics_node = Node(
        package="rosflight_sim",
        executable="standalone_dynamics",
        name='standalone_dynamics',
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, dynamics_param_file]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_vimfly_arg,
            environment_file_arg,
            lidar_output_frame_arg,
            simulator_launch_include,
            common_nodes_include,
            mr_forces_moments_node,
            standalone_dynamics_node,
        ]
    )
