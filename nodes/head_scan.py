#! /usr/bin/env python3

import numpy as np
import time
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState
from stretch_teleop_interface_msgs.action import HeadScan

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.server import ServerGoalHandle

class HeadScanActionServer(Node):
    def __init__(self):
        super().__init__('head_scan_server')
        self.loop_rate = self.create_rate(1, self.get_clock())

        self._server = ActionServer(self, HeadScan, 'head_scan', execute_callback=self.execute_callback, handle_accepted_callback=self.handle_accepted_callback)
        self.goal = HeadScan.Goal()
        self.feedback = HeadScan.Feedback()
        self.result = HeadScan.Result()
        self.tilt_range = [-2.01, 0.48]
        self.pan_range = [-4.07, 1.74]

        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "stretch_controller/follow_joint_trajectory",
        )

        self.aruco_marker_array = self.create_subscription(MarkerArray, 'aruco/marker_array', self.aruco_callback, 10)
        self.joint_states = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        # self._server.start()

    def aruco_callback(self, msg):
        self.markers = msg.markers
    
    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == "joint_head_pan":
                self.pan = position
            if name == "joint_head_tilt":
                self.tilt = position

    def set_pan_tilt_camera(self, pan, tilt):
        point = JointTrajectoryPoint()
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        point.positions = [pan, tilt]

        head_goal = FollowJointTrajectory.Goal()
        head_goal.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
        head_goal.trajectory.points = [point]
        head_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        head_goal.trajectory.header.frame_id = 'base_link'

        future = self._trajectory_client.send_goal_async(head_goal)
        # rclpy.spin_until_future_complete(self, future)

    def scan(self, pan_angles, tilt_angles):
        for tilt in tilt_angles:
            pan_angles = pan_angles[::-1]
            for pan in pan_angles:
                self.set_pan_tilt_camera(pan, tilt)
                time.sleep(0.5)
                for marker in self.markers:
                    print(self.aruco_name, marker.text)
                    if self.aruco_name == marker.text:
                        return True 
        
        return False
    
    def handle_accepted_callback(self, goal_handle):
        # with self._goal_lock:
        #     # This server only allows one goal at a time
        #     if self._goal_handle is not None and self._goal_handle.is_active:
        #         self.get_logger().info('Aborting previous goal')
        #         # Abort the existing goal
        #         self._goal_handle.abort()
        #     self._goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal: ServerGoalHandle):
        print('in callback')
        result = HeadScan.Result()
        self.goal = goal
        self.aruco_name = self.goal.request.name
        for marker in self.markers:
            if self.aruco_name == marker.text:
                result.success = True
                goal.succeed()
                return result

        # Scan in the local area
        pan_angles = [
            max(self.pan_range[0], self.pan - 0.5), 
            max(self.pan_range[0], self.pan - 0.25),
            min(self.pan_range[1], self.pan + 0.25),
            min(self.pan_range[1], self.pan + 0.5)]
        tilt_angles = [
            max(self.tilt_range[0], self.tilt - 0.5), 
            max(self.tilt_range[0], self.tilt - 0.25),
            min(self.tilt_range[1], self.tilt + 0.25),
            min(self.tilt_range[1], self.tilt + 0.5)]
        
        if goal.is_cancel_requested:
            goal.canceled()
            return result
        
        marker_found = self.scan(pan_angles, tilt_angles)
        if marker_found:
            result.success = True
            goal.succeed()
            print('returning')
            return result
        
        # Scan the surroundings        
        pan_angles = np.linspace(self.pan_range[0], self.pan_range[1], 15)
        tilt_angles = np.linspace(self.tilt_range[0], self.tilt_range[1], 5)
        marker_found = self.scan(pan_angles, tilt_angles)
        if not marker_found:
            self.result.success = False
            goal.succeed()
            return self.result
        
        print('succeed')
        goal.succeed()
        result.success = True
        return result
    
if __name__ == '__main__':
    rclpy.init()
    node = HeadScanActionServer()
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()