#!/usr/bin/env python3
""" Action server for opening doors.

Author: Charlie Street
Owner: Charlie Street
"""

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from door_msgs.action import OpenDoor
from door_msgs.srv import SetDoor
import rclpy.duration
from rclpy.node import Node
import rclpy


class OpenDoorServer(Node):
    """An action server which opens doors after a time delay.

    Attributes:
        _open_delay: The seconds to wait before calling the action client
        _set_door_client: The service client for setting doors
        _action_server: The door opening action server
    """

    def __init__(self):
        """Initialise the action server."""
        super().__init__("open_door_server")

        self.declare_parameter("open_delay", rclpy.Parameter.Type.DOUBLE)
        self._open_delay = self.get_parameter("open_delay").value

        self._set_door_client = self.create_client(SetDoor, "set_door")
        self._set_door_client.wait_for_service()
        self.get_logger().info("Set Door Client Active")

        self._action_server = ActionServer(
            self,
            OpenDoor,
            "open_door",
            execute_callback=self._execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_logger().info("Open Door Server Started")

    def destroy(self):
        """Destroy action server."""
        self._action_server.destroy()
        self._set_door_client.destroy()
        super().destroy_node()

    def _execute_callback(self, goal_handle):
        """Open the door after a delay.

        Args:
            goal_handle: The handle for the action goal

        Returns:
            The action result
        """

        self.get_logger().info("Open Door Request Received. Sleeping...")
        dur = rclpy.duration.Duration(seconds=self._open_delay)
        self.get_clock().sleep_for(dur)
        self.get_logger().info("Open Door Awake Again, Calling Client")

        req = SetDoor.Request()
        req.door = goal_handle.request.door
        req.status = "open"

        future = self._set_door_client.call_async(req)
        self.executor.spin_until_future_complete(future)
        set_door_result = future.result()

        # Pass through service result
        open_door_result = OpenDoor.Result()
        open_door_result.success = set_door_result.success

        if open_door_result.success:
            self.get_logger().info("Door Opened")
            goal_handle.succeed()
        else:
            self.get_logger().info("Door Not Opened")
            goal_handle.abort()

        return open_door_result


def main(args=None):
    rclpy.init(args=args)

    action_server = OpenDoorServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)
    action_server.destroy_node()
    rclpy.shutdown()
