#!/usr/bin/env python3
""" Launch all navigation functionality for the turtlebot in the bookstore.

Author: Charlie Street
Owner: Charlie Street
"""

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
import os


def generate_launch_description():

    turtlebot_nav2_dir = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"), "launch"
    )
    top_nav_dir = os.path.join(
        get_package_share_directory("topological_navigation"), "launch"
    )

    bookstore_root = get_package_share_directory("turtlebot_bookstore_sim")

    # Set turtlebot model env var
    set_turtle_model = SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle")

    map_file = os.path.join(bookstore_root, "maps/bookstore.yaml")
    top_map_file = os.path.join(bookstore_root, "maps/bookstore_top_map.yaml")
    # This is a modded version of the standard turtlebot nav params
    # I've set it to set an initial robot pose for AMCL
    # There may also be minor adjustments to obstacle inflation
    param_file = os.path.join(bookstore_root, "params/custom_waffle_nav_params.yaml")

    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_nav2_dir, "navigation2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "map": map_file,
            "params_file": param_file,
        }.items(),
    )

    start_top_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(top_nav_dir, "topological_navigation.launch.py")
        ),
        launch_arguments={
            "map": top_map_file,
            "pose_topic": "amcl_pose",
            "viz": "true",
        }.items(),
    )

    # Add the launch actions
    ld = LaunchDescription()
    ld.add_action(set_turtle_model)
    ld.add_action(start_nav2)
    ld.add_action(start_top_nav)

    return ld
