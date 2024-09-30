#!/usr/bin/env python3
""" Node which manages the doors in the bookstore.

Author: Charlie Street
Owner: Charlie Street
"""

from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from door_msgs.srv import CheckDoor, SetDoor
from rclpy.node import Node
from threading import Lock
from shapely import Point
import rclpy
import yaml

# Offset for RViz door markers
MARKER_OFFSET = 6004


class DoorManager(Node):
    """A node which manages doors in the environment.

    The doors status can be checked and set.

    Attributes:
        _status_lock: A lock on the door status
        _door_pos: A dictionary from door name to (x,y) point
        _door_status: A dictionary from door name to current door status
        _door_markers: A dictionary from door name to corresponding RViz Marker
        _marker_pub: A publisher for door markers in RViz
        _check_door: A service which returns the current door status
        _set_door: A service which sets the status of a door
    """

    def __init__(self):
        super().__init__("door_manager")

        self.declare_parameter("door_yaml", rclpy.Parameter.Type.STRING)
        self.declare_parameter("initial_status_list", rclpy.Parameter.Type.STRING_ARRAY)
        self._status_lock = Lock()

        with open(self.get_parameter("door_yaml").value, "r") as yaml_in:
            door_list = yaml.load(yaml_in, Loader=yaml.FullLoader)

        init_status_list = self.get_parameter("initial_status_list").value

        with self._status_lock:
            self._initialise_doors(door_list, init_status_list)

        # Publish markers
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "door_markers", qos_profile=latching_qos
        )
        self._initialise_rviz_markers()

        self._check_door = self.create_service(
            CheckDoor, "check_door", self._check_door_callback
        )

        self._set_door = self.create_service(
            SetDoor, "set_door", self._set_door_callback
        )

        self.get_logger().info("Door Manager Started.")

    def _initialise_doors(self, door_list, init_status_list):
        """Initialise the doors.

        Args:
            door_list: A list of dictionaries with door names and coordinates
            init_status_list: A list of initial door statuses (open or closed)
        """
        self._door_pos = {}
        self._door_status = {}

        for i in range(len(door_list)):
            door = door_list[i]
            self._door_pos[door["name"]] = Point(door["x"], door["y"])
            assert init_status_list[i] == "open" or init_status_list[i] == "closed"
            self._door_status[door["name"]] = init_status_list[i]

    def _initialise_rviz_markers(self):
        """Initialise the markers for each of the doors and publish them."""
        self._door_markers = {}
        marker_array = MarkerArray()

        marker_id = 0
        for door in self._door_pos:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = MARKER_OFFSET + marker_id
            marker.type = 1  # CUBE
            marker.action = 0
            marker.pose.position.x = self._door_pos[door].x
            marker.pose.position.y = self._door_pos[door].y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.3
            marker.scale.y = 1.5
            marker.scale.z = 0.5
            marker.color.r = 0.72 if self._door_status[door] == "closed" else 0.3
            marker.color.g = 0.11 if self._door_status[door] == "closed" else 0.69
            marker.color.b = 0.11 if self._door_status[door] == "closed" else 0.31
            marker.color.a = 0.8
            marker_id += 1
            self._door_markers[door] = marker
            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)
        self.get_logger().info("Initial Door Markers Published.")

    def _update_marker(self, door, status):
        """Update the marker colour to reflect its new status.

        Args:
            door: The door name
            status: The status of the door
        """

        assert status == "open" or status == "closed"

        # Switch colours - green means open, red means closed
        self._door_markers[door].color.r = (
            0.72 if self._door_status[door] == "closed" else 0.3
        )
        self._door_markers[door].color.g = (
            0.11 if self._door_status[door] == "closed" else 0.69
        )
        self._door_markers[door].color.b = (
            0.11 if self._door_status[door] == "closed" else 0.31
        )

        # Resend everything to make sure all markers still show up
        marker_array = MarkerArray()
        for door in self._door_markers:
            marker_array.markers.append(self._door_markers[door])

        self._marker_pub.publish(marker_array)
        self.get_logger().info("{} Marker Updated to {}".format(door, status))

    def _check_door_callback(self, request, response):
        """Return the status of a given door.

        Args:
            request: Contains the door to be checked
            response: Contains the status of the door

        Returns:
            The filled in response object
        """
        self.get_logger().info("Check Request at Door {}".format(request.door))
        with self._status_lock:
            response.status = self._door_status[request.door]
            return response

    def _set_door_callback(self, request, response):
        """Set the status of a given door.

        Args:
            request: Contains the door to be set and the corresponding status
            response: Contains the success flag of the action

        Returns:
            The filled in response object
        """
        self.get_logger().info(
            "Set Request for Door {}; Status: {}".format(request.door, request.status)
        )
        assert request.door in self._door_status
        assert request.status == "open" or request.status == "closed"
        with self._status_lock:
            self._door_status[request.door] = request.status
            self._update_marker(request.door, request.status)
            response.success = True
            return response


def main(args=None):
    rclpy.init(args=args)

    door_manager = DoorManager()
    rclpy.spin(door_manager)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
