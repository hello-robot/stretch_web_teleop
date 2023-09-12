#! /usr/bin/env python3

import rospy
import actionlib
import numpy as np
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState
from stretch_teleop_interface.msg import HeadScanAction, HeadScanGoal, HeadScanFeedback, HeadScanResult

class HeadScanServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('head_scan', HeadScanAction, self.execute_callback, False)
        self.goal = HeadScanGoal()
        self.feedback = HeadScanFeedback()
        self.result = HeadScanResult()
        self.tilt_range = [-1.0, 0.0]
        self.pan_range = [-4.07, 1.74]
        self.preempt = False

        self.trajectory_client = actionlib.SimpleActionClient(
            "stretch_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        self.aruco_marker_array = rospy.Subscriber('aruco/marker_array', MarkerArray, self.aruco_callback)
        self.joint_states = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        self.server.start()
    
    def preempt_callback(self):
        print('preempt callback')
        self.result.success = False
        self.trajectory_client.cancel_all_goals()
        self.server.set_preempted(self.result)

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
        point.time_from_start = rospy.Duration(1.0)
        point.positions = [pan, tilt]

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
        head_goal.trajectory.points = [point]
        head_goal.trajectory.header.stamp = rospy.Time.now()
        head_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal_and_wait(head_goal)
    
    def scan(self, pan_angles, tilt_angles):
        for tilt in tilt_angles:
            pan_angles = pan_angles[::-1]
            for pan in pan_angles:
                self.set_pan_tilt_camera(pan, tilt)
                if self.server.is_preempt_requested(): 
                    return False
                rospy.sleep(0.2)
                for marker in self.markers:
                    if self.aruco_name == marker.text:
                        return True 
        
        return False
    
    def execute_callback(self, goal):
        if self.server.is_preempt_requested():
            self.preempt_callback()
            return

        self.goal = goal
        self.aruco_name = self.goal.name
        for marker in self.markers:
            if self.aruco_name == marker.text:
                self.result.success = True
                self.server.set_succeeded(self.result)
                return

        # Scan in the local area
        pan_angles = [
            max(self.pan_range[0], self.pan - 0.5), 
            max(self.pan_range[0], self.pan - 0.25),
            min(self.pan_range[1], self.pan + 0.25),
            min(self.pan_range[1], self.pan + 0.5)]
        tilt_angles = [
            max(self.tilt_range[0], self.tilt + 0.5), 
            max(self.tilt_range[0], self.tilt + 0.25),
            min(self.tilt_range[1], self.tilt - 0.25),
            min(self.tilt_range[1], self.tilt - 0.5)]
        
        marker_found = self.scan(pan_angles, tilt_angles)
        if marker_found:
            self.result.success = True
            self.server.set_succeeded(self.result)
            return
        
        # Scan the surroundings        
        pan_angles = np.linspace(self.pan_range[0], self.pan_range[1], 15)
        tilt_angles = np.linspace(self.tilt_range[0], self.tilt_range[1], 5)
        marker_found = self.scan(pan_angles, tilt_angles)
        if self.server.is_preempt_requested():
            self.preempt_callback()
        elif not marker_found:
            self.result.success = False
            self.server.set_aborted(self.result)
            return
        elif marker_found:
            self.result.success = True
            self.server.set_succeeded(self.result)
            return
        
if __name__ == '__main__':
    rospy.init_node('head_scan_server')
    server = HeadScanServer()
    rospy.spin()