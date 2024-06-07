#!/usr/bin/env python3

# Standard Imports
import sys
import threading
from typing import List, Optional

# Third-party Imports
import rclpy
import tf2_ros
from configure_video_streams import ConfigureVideoStreams

# from move_to_pregrasp import MoveToPregrasp
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# from stretch_controls import StretchControls

# Local Imports


class WebTeleopNode(Node):
    """
    This node runs the ROS-side behavior for the web teleop interface. Currently,
    this includes:
      1. Rotating/cropping/masking all camera feeds.
      2. Exposing the move_to_pregrasp action.

    The reason for combining all this functionality into one node is to prevent
    multiple subscriptions to topics---particularly camera topics---as those can
    cause high CPU load.
    """

    def __init__(
        self,
        image_params_file: str,
        has_beta_teleop_kit: bool,
        tf_buffer_secs: float = 12.0,
    ):
        """
        Initialize the node.

        Parameters
        ----------
        image_params_file: The path to the YAML file containing the image parameters.
        has_beta_teleop_kit: Whether the robot has the beta teleop kit.
        tf_buffer_secs: The number of seconds to cache transforms for.
        """
        super().__init__("web_teleop")

        # Create the shared resources that each component of this node needs.
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=tf_buffer_secs))
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create the components of this node.
        # self.stretch_controls = StretchControls(self)
        self.video_streams = ConfigureVideoStreams(
            self, image_params_file, has_beta_teleop_kit, self.tf_buffer
        )
        # self.move_to_pregrasp = MoveToPregrasp(self, self.video_streams, self.tf_buffer)

    def initialize(self):
        """
        Initialize the components of this node. This function is used for any
        initialization that requires ROS spinning to be occurring in the background,
        e.g., invoking a service during initialization.
        """
        # self.stretch_controls.initialize()
        self.video_streams.initialize()
        # self.move_to_pregrasp.initialize()


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    image_params_file = args[0]
    has_beta_teleop_kit = args[1] == "True"

    web_teleop = WebTeleopNode(image_params_file, has_beta_teleop_kit)
    web_teleop.get_logger().info("Created Web Teleop Node.")

    # Use a MultiThreadedExecutor so that subscriptions, actions, etc. can be
    # processed in parallel.
    executor = MultiThreadedExecutor()

    # Spin in the background, as the node initializes
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(web_teleop,),
        kwargs={"executor": executor},
        daemon=True,
    )
    spin_thread.start()

    # Initialize the node
    web_teleop.initialize()
    web_teleop.get_logger().info("Initialized Web Teleop Node.")

    # Spin in the foreground
    spin_thread.join()

    web_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
