#!/usr/bin/env python3
""" Launch file for all door-related behaviours.

Author: Charlie Street
Owner: Charlie Street
"""

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
import os


def generate_launch_description():

    # Set sim time
    set_sim_time = SetParameter(name="use_sim_time", value=True)

    # Get door file
    bookstore_root = get_package_share_directory("turtlebot_bookstore_sim")
    bookstore_door_path = os.path.join(
        bookstore_root, "maps/bookstore/bookstore_door_map.yaml"
    )

    # All launch args
    door_path = LaunchConfiguration("door_yaml")
    init_status_file = LaunchConfiguration("initial_status_file")
    initial_status_idx = LaunchConfiguration("initial_status_index")
    open_delay = LaunchConfiguration("open_delay")

    door_arg = DeclareLaunchArgument("door_yaml", default_value=bookstore_door_path)
    init_status_file_arg = DeclareLaunchArgument(
        "initial_status_file", default_value="not_set"
    )
    initial_status_idx_arg = DeclareLaunchArgument(
        "initial_status_index", default_value="-1"
    )
    open_delay_arg = DeclareLaunchArgument("open_delay", default_value="30.0")

    door_manager_node = Node(
        package="turtlebot_bookstore_sim",
        executable="door_manager",
        name="door_manager",
        parameters=[
            {
                "door_yaml": door_path,
                "initial_status_file": init_status_file,
                "initial_status_index": initial_status_idx,
            }
        ],
        output="screen",
    )

    open_door_node = Node(
        package="turtlebot_bookstore_sim",
        executable="open_door_server",
        name="open_door_server",
        parameters=[{"open_delay": open_delay}],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(set_sim_time)
    ld.add_action(door_arg)
    ld.add_action(init_status_file_arg)
    ld.add_action(initial_status_idx_arg)
    ld.add_action(open_delay_arg)
    ld.add_action(door_manager_node)
    ld.add_action(open_door_node)

    return ld
