#!/usr/bin/env python3
""" A node for policy execution in the bookstore.

This policy executor can run data collection or policy execution, but is
very tied to the bookstore environment.

Author: Charlie Street
Owner: Charlie Street
"""

from topological_navigation.topological_map import TopologicalMap
from refine_plan.models.state_factor import StateFactor
from topological_msgs.action import NavigateEdge
from refine_plan.models.state import State
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from door_msgs.action import OpenDoor
from door_msgs.srv import CheckDoor
from pymongo import MongoClient
from datetime import datetime
from rclpy.node import Node
import random
import rclpy
import yaml


class BookstorePolicyExecutor(Node):
    """A node for behaviour execution in the bookstore environment.

    Attributes:
        _db_collection: The MongoDB collection for data logging
        _top_map: The topological map
        _doors_on_edge: A dictionary from edge to door on that edge
        _edge_client: The action client for edge navigation
        _check_door_client: The client for the check door service
        _open_door_client: The action client for opening doors
        _run_id: The ID for this run
        _policy_fn: A function
    """

    def __init__(self):
        """Initialise the policy executor."""
        super().__init__("policy_executor")

        self.declare_parameter("db_connection_string", rclpy.Parameter.Type.STRING)
        self.declare_parameter("db_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("db_collection", rclpy.Parameter.Type.STRING)
        self.declare_parameter("map", rclpy.Parameter.Type.STRING)
        self.declare_parameter("doors_on_edge", rclpy.Parameter.Type.STRING)

        self._run_id = random.getrandbits(32)
        self._policy_fn = self._rand_action  # TODO: Generalise
        self._top_map = TopologicalMap(self.get_parameter("map").value)

        with open(self.get_parameter("doors_on_edge").value, "r") as yaml_in:
            self._doors_on_edge = yaml.load(yaml_in, Loader=yaml.FullLoader)

        connect_str = self.get_parameter("db_connection_string").value
        db_name = self.get_parameter("db_name").value
        db_collection = self.get_parameter("db_collection").value
        self._db_collection = MongoClient(connect_str)[db_name][db_collection]
        self.get_logger().info("Connected to MongoDB")

        self._edge_client = ActionClient(self, NavigateEdge, "edge_navigation")
        self._edge_client.wait_for_server()
        self.get_logger().info("Edge Navigation Client Active")

        self._check_door_client = self.create_client(CheckDoor, "check_door")
        self._check_door_client.wait_for_service()
        self.get_logger().info("Check Door Client Active")

        self._open_door_client = ActionClient(self, OpenDoor, "open_door")
        self._open_door_client.wait_for_server()
        self.get_logger().info("Open Door Client Active")

        self.get_logger().info("Policy Executor Setup")

    def _create_initial_state(self):
        """Creates the initial state for policy execution.

        Returns:
            The initial state
        """
        loc_sf = StateFactor("location", list(self._top_map._nodes.keys()))
        doors = set(self._doors_on_edge.values())
        door_sfs = []
        for door in doors:
            door_sfs.append(StateFactor(door, ["unknown", "closed", "open"]))

        state_dict = {loc_sf: "v1"}
        for sf in door_sfs:
            state_dict[sf] = "unknown"

        return State(state_dict)

    def _enabled_actions(self, state):
        """Return the enabled actions in a state.

        Args:
            state: The current state

        Returns:
            A list of enabled actions
        """
        enabled_actions = set([])

        door_locs = set([d[:-5] for d in self._doors_on_edge.values()])
        current_loc = state["location"]

        # Door actions
        for loc in door_locs:
            if current_loc == loc:
                if state["{}_door".format(loc)] == "closed":
                    enabled_actions.add("open_door")
                elif state["{}_door".format(loc)] == "unknown":
                    enabled_actions.add("check_door")

        # Navigation
        for edge in self._top_map.edges_from_node(current_loc):
            if edge not in self._doors_on_edge:  # No door to worry about
                enabled_actions.add(edge)
            elif state[self._doors_on_edge[edge]] == "open":
                enabled_actions.add(edge)

        return list(enabled_actions)

    def _rand_action(self, state):
        """Select a random enabled action for data collection.

        Args:
            state: Unused

        Returns:
            The action to be executed
        """
        return random.choice(self._enabled_actions(state))

    def _initial_policy(self, state):
        """The initial policy which ignores uncertainty.

        Representative of the initial BT.

        Here, we follow the shortest path and go through any door it sees.

        Args:
            state: The current state of the system
        """

        if state["location"] == "v1":
            return "e13"
        elif state["location"] == "v3" and state["v3_door"] == "unknown":
            return "check_door"
        elif state["location"] == "v3" and state["v3_door"] == "closed":
            return "open_door"
        elif state["location"] == "v3" and state["v3_door"] == "open":
            return "e36"
        elif state["location"] == "v6" and state["v6_door"] == "unknown":
            return "check_door"
        elif state["location"] == "v6" and state["v6_door"] == "closed":
            return "open_door"
        elif state["location"] == "v6" and state["v6_door"] == "open":
            return "e68"
        else:
            return None

    def _log_action(self, state, new_state, action, start, end):
        """Log an action to the mongoDB database.

        Args:
            state: The predecessor state
            new_state: The successor state
            action: The executed action
            start: The start time
            end: The end time
        """
        doc = {}
        doc["run_id"] = self._run_id
        doc["option"] = action
        doc["date_started"] = float(start.nanoseconds) / 1e9
        doc["date_finished"] = float(end.nanoseconds) / 1e9
        doc["duration"] = float((end - start).nanoseconds) / 1e9
        doc["_meta"] = {"inserted_at": datetime.now()}

        for sf in state._state_dict:
            doc["{}0".format(sf)] = state[sf]

        for sf in new_state._state_dict:
            doc["{}t".format(sf)] = new_state[sf]

        self._db_collection.insert_one(doc)

    def _check_door(self, state):
        """Check the status of a door.

        Args:
            state: The current state

        Returns:
            The updated state
        """
        door = "{}_door".format(state["location"])
        door_req = CheckDoor.Request()
        door_req.door = door

        future = self._check_door_client.call_async(door_req)
        start = self.get_clock().now()
        rclpy.spin_until_future_complete(self, future)
        end = self.get_clock().now()
        result = future.result()

        new_state_dict = {}  # Update door status
        for sf in state._state_dict:
            new_state_dict[state._sf_dict[sf]] = (
                result.status if sf == door else state._state_dict[sf]
            )

        new_state = State(new_state_dict)
        self._log_action(state, new_state, "check_door", start, end)
        return new_state

    def _open_door(self, state):
        """Open a door.

        Args:
            state: The current state

        Returns:
            The updated state
        """
        door = "{}_door".format(state["location"])
        door_goal = OpenDoor.Goal()
        door_goal.door = door

        future = self._open_door_client.send_goal_async(door_goal)
        rclpy.spin_until_future_complete(self, future)
        start = self.get_clock().now()
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Open Door Action Not Accepted")
            return None

        result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        end = self.get_clock().now()

        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error("Open Door Action Failed")
            return None
        else:
            new_state_dict = {}
            for sf in state._state_dict:  # Set door to open
                new_state_dict[state._sf_dict[sf]] = (
                    "open" if sf == door else state._state_dict[sf]
                )

            new_state = State(new_state_dict)
            self._log_action(state, new_state, "open_door", start, end)

            return new_state

    def _edge_navigation(self, state, action):
        """Navigate along a topological edge.

        Args:
            state: The current state
            action: The current action

        Returns:
            The updated state
        """
        edge_goal = NavigateEdge.Goal()
        edge_goal.edge_id = action

        future = self._edge_client.send_goal_async(edge_goal)
        rclpy.spin_until_future_complete(self, future)
        start = self.get_clock().now()
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Edge Navigation Action Not Accepted")
            return None

        result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        end = self.get_clock().now()

        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error("Edge Navigation Action Failed")
            return None
        else:
            new_loc = result.result().result.dest
            new_state_dict = {}

            for sf in state._state_dict:  # Update location
                new_state_dict[state._sf_dict[sf]] = (
                    new_loc if sf == "location" else state._state_dict[sf]
                )

            new_state = State(new_state_dict)
            self._log_action(state, new_state, action, start, end)

            return new_state

    def execute_policy(self):
        """Execute the policy until a termination condition is satisfied."""

        current_state = self._create_initial_state()

        while True:  # TODO: Generalise
            action = self._policy_fn(current_state)
            assert action in self._enabled_actions(current_state)

            self.get_logger().info("Executing {} in {}".format(action, current_state))

            if action == "open_door":
                current_state = self._open_door(current_state)
            elif action == "check_door":
                current_state = self._check_door(current_state)
            else:  # Edge action
                current_state = self._edge_navigation(current_state, action)

            if current_state is None:  # Error case
                self.get_logger().error("Error During Policy Execution. Exiting.")
                return


def main(args=None):
    rclpy.init(args=args)

    policy_exec = BookstorePolicyExecutor()

    # Execute the policy and then shut down
    policy_exec.execute_policy()
    rclpy.shutdown()
