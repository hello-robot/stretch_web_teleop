#!/usr/bin/env python3

# Standard imports
import threading
import time
from typing import List

# Third-party imports
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# Local Imports
from stretch_tablet_interfaces.action import ShowTablet


class DummyShowTablet(Node):
    def __init__(
        self,
        send_feedback_hz: float = 10.0,
        show_tablet_secs: float = 5,
    ):
        """
        Initialize the DummyShowTablet action node.

        Parameters
        ----------
        name: The name of the action server.
        send_feedback_hz: The target frequency at which to send feedback.
        show_tablet_secs: The time in seconds to execute the action.
        """
        super().__init__("dummy_show_tablet")

        self.send_feedback_hz = send_feedback_hz
        self.show_tablet_secs = show_tablet_secs

        self.active_goal_request = None

        self._action_server = ActionServer(
            self,
            ShowTablet,
            "/show_tablet",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request: ShowTablet.Goal) -> GoalResponse:
        """
        Accept a goal if this action does not already have an active goal,
        else reject.

        Parameters
        ----------
        goal_request: The goal request message.
        """
        self.get_logger().info("Received goal request")
        if self.active_goal_request is None:
            self.get_logger().info("Accepting goal request")
            self.active_goal_request = goal_request
            return GoalResponse.ACCEPT
        self.get_logger().info("Rejecting goal request")
        return GoalResponse.REJECT

    def cancel_callback(self, _: ServerGoalHandle) -> CancelResponse:
        """
        Always accept client requests to cancel the active goal. Note that this
        function should not actually implement the cancel; that is handled in
        `execute_callback`

        Parameters
        ----------
        goal_handle: The goal handle.
        """
        self.get_logger().info("Received cancel request, accepting")
        return CancelResponse.ACCEPT

    def dummy_show_tablet(self, show_tablet_success: List[bool]) -> None:
        """
        A dummy thread for showing the tablet. This thread will sleep for
        `self.dummy_plan_time` sec and then set the plan to None.

        Parameters
        ----------
        show_tablet_success: A list to store whether the show tablet was successful.
        """
        time.sleep(self.show_tablet_secs)
        show_tablet_success[0] = True

    async def execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> ShowTablet.Result:
        """
        Execute the dummy action.

        Parameters
        ----------
        goal_handle: The goal handle.

        Returns
        -------
        result: The result message.
        """
        self.get_logger().info("Executing goal...%s" % (goal_handle.request,))

        # Load the feedback parameters
        feedback_rate = self.create_rate(self.send_feedback_hz)
        feedback_msg = ShowTablet.Feedback()

        # Start the show tablet thread
        show_tablet_success = [False]
        show_tablet_thread = threading.Thread(
            target=self.dummy_show_tablet, args=(show_tablet_success,), daemon=True
        )
        show_tablet_thread.start()
        show_tablet_start_time = self.get_clock().now()

        # Monitor the show tablet thread and send feedback
        while rclpy.ok():
            # Check if there is a cancel request
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                goal_handle.canceled()
                result = ShowTablet.Result()
                result.status = result.STATUS_CANCELED
                self.active_goal_request = None  # Clear the active goal
                return result

            # Check if the planning thread has finished
            if not show_tablet_thread.is_alive():
                if show_tablet_success[0]:  # Plan succeeded
                    self.get_logger().info("Show tablet succeeded")
                    # Succeed the goal
                    goal_handle.succeed()
                    result = ShowTablet.Result()
                    result.status = result.STATUS_SUCCESS
                    self.active_goal_request = None  # Clear the active goal
                    return result
                else:  # Plan failed
                    self.get_logger().info("Show tablet failed, aborting")
                    # Abort the goal
                    goal_handle.abort()
                    result = self.action_class.Result()
                    result.status = result.STATUS_PLANNING_FAILED
                    self.active_goal_request = None  # Clear the active goal
                    return result

            # Send feedback
            feedback_msg.current_state = int(
                (self.get_clock().now() - show_tablet_start_time).nanoseconds / 1.0e9
            )
            self.get_logger().info("Feedback: %s" % feedback_msg)
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for the specified feedback rate
            feedback_rate.sleep()

        # If we get here, something went wrong
        self.get_logger().info("Unknown error, aborting")
        goal_handle.abort()
        result = ShowTablet.Result()
        result.status = result.STATUS_UNKNOWN
        self.active_goal_request = None  # Clear the active goal
        return result


def main(args=None):
    rclpy.init(args=args)

    node = DummyShowTablet()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)


if __name__ == "__main__":
    main()
