#!/usr/bin/env python3
""" Launch file for the policy executor.

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

    root_dir = get_package_share_directory("turtlebot_bookstore_sim")
    top_map_file = os.path.join(root_dir, "maps/bookstore_top_map.yaml")
    doors_on_edge_file = os.path.join(root_dir, "maps/bookstore_doors_on_edge.yaml")

    # All launch args
    db_connection_string = LaunchConfiguration("db_connection_string")
    db_name = LaunchConfiguration("db_name")
    db_collection = LaunchConfiguration("db_collection")
    map_path = LaunchConfiguration("map")
    doors_on_edge = LaunchConfiguration("doors_on_edge")
    mode = LaunchConfiguration("mode")

    db_connection_arg = DeclareLaunchArgument(
        "db_connection_string", default_value="localhost:27107"
    )
    db_name_arg = DeclareLaunchArgument("db_name", default_value="refine-plan")
    db_collection_arg = DeclareLaunchArgument("db_collection")
    map_path_arg = DeclareLaunchArgument("map", default_value=top_map_file)
    doors_on_edge_arg = DeclareLaunchArgument(
        "doors_on_edge", default_value=doors_on_edge_file
    )
    mode_arg = DeclareLaunchArgument("mode")

    # Policy executor node
    policy_exec_node = Node(
        package="turtlebot_bookstore_sim",
        executable="policy_executor",
        name="policy_executor",
        parameters=[
            {
                "db_connection_string": db_connection_string,
                "db_name": db_name,
                "db_collection": db_collection,
                "map": map_path,
                "doors_on_edge": doors_on_edge,
                "mode": mode,
            }
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(set_sim_time)
    ld.add_action(db_connection_arg)
    ld.add_action(db_name_arg)
    ld.add_action(db_collection_arg)
    ld.add_action(map_path_arg)
    ld.add_action(doors_on_edge_arg)
    ld.add_action(mode_arg)
    ld.add_action(policy_exec_node)

    return ld
