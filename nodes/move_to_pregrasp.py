#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from stretch_web_teleop.action import MoveToPregrasp


class MoveToPregraspNode(Node):
    def __init__(self):
        super().__init__("move_to_pregrasp")
        self._action_server = ActionServer(
            self, MoveToPregrasp, "move_to_pregrasp", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Got request {goal_handle.request}")
        goal_handle.succeed()
        result = MoveToPregrasp.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    move_to_pregrasp = MoveToPregraspNode()
    move_to_pregrasp.get_logger().info("Initialized!")

    rclpy.spin(move_to_pregrasp)


if __name__ == "__main__":
    main()
