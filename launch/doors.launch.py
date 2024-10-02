#!/usr/bin/env python3
""" Launch file for all door-related behaviours.

Author: Charlie Street
Owner: Charlie Street
"""

from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # All launch args
    door_path = LaunchConfiguration("door_yaml")
    init_door_status = LaunchConfiguration("initial_status_list")
    open_delay = LaunchConfiguration("open_delay")

    door_arg = DeclareLaunchArgument("door_yaml")
    init_status_arg = DeclareLaunchArgument("initial_status_list")
    open_delay_arg = DeclareLaunchArgument("open_delay", default_value="10.0")

    door_manager_node = Node(
        package="turtlebot_bookstore_sim",
        executable="door_manager",
        name="door_manager",
        parameters=[{"door_yaml": door_path, "initial_status_list": init_door_status}],
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
    ld.add_action(door_arg)
    ld.add_action(init_status_arg)
    ld.add_action(open_delay_arg)
    ld.add_action(door_manager_node)
    ld.add_action(open_door_node)

    return ld
