#! /usr/bin/env python3

import rospy
import tf2_ros
import actionlib
import dynamic_reconfigure.client
from math import fabs, sqrt
from geometry_msgs.msg import TransformStamped, Transform
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray
import numpy as np
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import PoseStamped
from stretch_teleop_interface_msgs.msg import HeadScanAction, HeadScanGoal, NavigateToArucoAction, NavigateToArucoGoal, NavigateToArucoFeedback, NavigateToArucoResult
from stretch_teleop_interface_msgs.srv import RelativePose
from geometry_msgs.msg import Twist

class NavigateToArucoServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('navigate_to_aruco', NavigateToArucoAction, self.execute_callback, False)
        self.relative_pose_service = rospy.Service('get_relative_pose', RelativePose, self.get_relative_pose_callback)
        self.server.register_preempt_callback(self.cancel_goals)
        self.goal = NavigateToArucoGoal()
        self.feedback = NavigateToArucoFeedback()
        self.result = NavigateToArucoResult()
        self.num_tries = 0
        self.max_tries = 5
        self.preempt = False
        self.reconfigure_client = dynamic_reconfigure.client.Client("move_base")

        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=10)

        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        self.head_scan_client = actionlib.SimpleActionClient(
            "head_scan",
            HeadScanAction
        )
        
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base",
            MoveBaseAction
        )

        self.head_to_aruco_client = actionlib.SimpleActionClient(
            "head_to_aruco",
            HeadScanAction
        )

        self.server.start()
    
    def cancel_goals(self):
        print('cancel goals')
        self.head_scan_client.cancel_all_goals()
        self.move_base_client.cancel_all_goals()
        self.server.set_preempted()

    def preempt_callback(self):
        print('preempt callback')
        self.feedback.state = 'Aruco navigation cancelled!'
        self.feedback.alert_type = "error"
        self.server.publish_feedback(self.feedback)
        
    def broadcast_tf(self, trans, name, ref):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = ref
        t.child_frame_id = name
        t.transform = trans
        return t
    
    def get_relative_pose_callback(self, req):
        self.aruco_name = req.name
        self.feedback.state = 'Scanning for marker...'
        self.feedback.alertType = "info"
        self.server.publish_feedback(self.feedback)
        head_scan_goal = HeadScanGoal()
        head_scan_goal.name = self.aruco_name
        result = self.head_scan_client.send_goal_and_wait(head_scan_goal)

        if not self.head_scan_client.get_result(): 
            self.feedback.state = 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
            self.feedback.alertType = "error"
            self.server.publish_feedback(self.feedback)
            self.server.set_aborted()
            return

        try:
            map_pose = self.tf2_buffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
            pose_stamped =  PoseStamped()
            pose_stamped.header = map_pose.header
            pose_stamped.pose.position.x = map_pose.transform.translation.x
            pose_stamped.pose.position.y = map_pose.transform.translation.y
            pose_stamped.pose.position.z = map_pose.transform.translation.z
            pose_stamped.pose.orientation.x = map_pose.transform.rotation.x
            pose_stamped.pose.orientation.y = map_pose.transform.rotation.y
            pose_stamped.pose.orientation.z = map_pose.transform.rotation.z
            pose_stamped.pose.orientation.w = map_pose.transform.rotation.w

            trans = self.tf2_buffer.transform(pose_stamped, self.aruco_name, rospy.Duration(1.0))
        
            transform = Transform()
            transform.translation.x = trans.pose.position.x
            transform.translation.y = trans.pose.position.y
            transform.translation.z = trans.pose.position.z
            transform.rotation.x = trans.pose.orientation.x
            transform.rotation.y = trans.pose.orientation.y
            transform.rotation.z = trans.pose.orientation.z
            transform.rotation.w = trans.pose.orientation.w

            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Could not publish pose to tf")
            pass

    def execute_callback(self, goal):
        self.reconfigure_client.update_configuration({"controller_frequency": 20.0})

        self.num_tries = 0
        self.goal = goal
        self.aruco_name = self.goal.name
        self.relative_pose = self.goal.pose

        self.feedback.state = 'Scanning for marker...'
        self.feedback.alert_type = "info"
        self.server.publish_feedback(self.feedback)
        head_scan_goal = HeadScanGoal()
        head_scan_goal.name = self.aruco_name
        result = self.head_scan_client.send_goal_and_wait(head_scan_goal)
        if self.server.is_preempt_requested():
            self.preempt_callback()
            return 
        
        if not self.head_scan_client.get_result(): 
            self.feedback.state = 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
            self.feedback.alert_type = "error"
            self.server.publish_feedback(self.feedback)
            self.server.set_aborted()
            return

        self.marker = self.tf2_buffer.lookup_transform('base_link', self.aruco_name, rospy.Time())
        self.map_to_aruco = self.tf2_buffer.lookup_transform('map', self.aruco_name, rospy.Time())
        self.aruco_tf = self.broadcast_tf(self.map_to_aruco.transform, self.aruco_name, 'map')
        rospy.loginfo("{} pose published to tf".format(self.aruco_name))

        self.navigate_to_marker()
    
    def average_transforms(self, transforms):
        trans = transforms[0]
        trans.transform.translation.z = 0
        angles = euler_from_quaternion([trans.transform.rotation.x,
                                        trans.transform.rotation.y,
                                        trans.transform.rotation.z,
                                        trans.transform.rotation.w])
        angle = angles[2]

        for i in range(1, len(transforms)):
            trans.transform.translation.x += transforms[i].transform.translation.x
            trans.transform.translation.y += transforms[i].transform.translation.y
            angles = euler_from_quaternion([transforms[i].transform.rotation.x,
                                            transforms[i].transform.rotation.y,
                                            transforms[i].transform.rotation.z,
                                            transforms[i].transform.rotation.w])
            angle += angles[2]

        trans.transform.translation.x /= len(transforms)
        trans.transform.translation.y /= len(transforms)
        angle /= len(transforms)
        
        q = quaternion_from_euler(0, 0, angle)
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]

        return trans
    
    def calculate_diff(self, trans1, trans2):
        angle1 = euler_from_quaternion([trans1.transform.rotation.x,
                                        trans1.transform.rotation.y,
                                        trans1.transform.rotation.z,
                                        trans1.transform.rotation.w])
        angle2 = euler_from_quaternion([trans2.transform.rotation.x,
                                        trans2.transform.rotation.y,
                                        trans2.transform.rotation.z,
                                        trans2.transform.rotation.w])
        diff = sqrt((trans1.transform.translation.x - trans2.transform.translation.x)**2 + \
                                (trans1.transform.translation.y - trans2.transform.translation.y)**2 + \
                                (angle1[2] - angle2[2])**2)
        return diff
    
    def navigate_to_marker(self):
        if self.server.is_preempt_requested():
            self.preempt_callback()
            return
        
        self.num_tries += 1
        tran = self.broadcast_tf(self.relative_pose, 'relative_pose', self.aruco_name)
        self.tf2_broadcaster.sendTransform(tran)
        transforms = []

        self.feedback.state = "Navigating to marker..."
        self.feedback.alert_type = "info"
        self.server.publish_feedback(self.feedback)

        while len(transforms) < 50:
            try:
                trans = self.tf2_buffer.lookup_transform('map', 'relative_pose', rospy.Time(), rospy.Duration(1.0))
                transforms.append(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Could not publish pose to tf")
                self.feedback.state = 'Navigation failed, please try again.'
                self.feedback.alert_type = "error"
                self.server.publish_feedback(self.feedback)
                self.server.set_aborted()
                return
                        
        self.relative_pose_tf = self.average_transforms(transforms)

        rospy.loginfo("Published relative pose")

        goal_pose = Pose()
        goal_pose.position.x = self.relative_pose_tf.transform.translation.x
        goal_pose.position.y = self.relative_pose_tf.transform.translation.y
        goal_pose.position.z = self.relative_pose_tf.transform.translation.z
        goal_pose.orientation.x = self.relative_pose_tf.transform.rotation.x
        goal_pose.orientation.y = self.relative_pose_tf.transform.rotation.y
        goal_pose.orientation.z = self.relative_pose_tf.transform.rotation.z
        goal_pose.orientation.w = self.relative_pose_tf.transform.rotation.w

        action_goal = MoveBaseGoal()
        action_goal.target_pose.header.frame_id = "map"
        action_goal.target_pose.header.stamp = rospy.Time.now()
        action_goal.target_pose.pose = goal_pose

        # head_scan_goal = HeadScanGoal()
        # head_scan_goal.name = self.aruco_name
        # self.head_to_aruco_client.send_goal(head_scan_goal)
        result = self.move_base_client.send_goal_and_wait(action_goal)
        # self.head_to_aruco_client.cancel_all_goals()

        self.move_base_aruco_callback(result)
    
    def rotate_to_place(self):
        goal_angles = euler_from_quaternion([self.relative_pose_tf.transform.rotation.x,
                                             self.relative_pose_tf.transform.rotation.y,
                                             self.relative_pose_tf.transform.rotation.z,
                                             self.relative_pose_tf.transform.rotation.w])
        goal_theta = goal_angles[2]

        diff = 0.5

        while fabs(diff) > 0.025:
            try:
                curr_pose = self.tf2_buffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
                curr_angles = euler_from_quaternion([curr_pose.transform.rotation.x,
                                                     curr_pose.transform.rotation.y,
                                                     curr_pose.transform.rotation.z,
                                                     curr_pose.transform.rotation.w])
                diff = goal_theta - curr_angles[2]
                print(diff)
                self.cmd_vel.angular.z = 0.75*diff
                self.cmd_vel_pub.publish(self.cmd_vel)            
            except:
                print('except')
                self.cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(self.cmd_vel)

        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)
        
    def move_base_aruco_callback(self, result):
        if self.server.is_preempt_requested():
            self.preempt_callback()
            return
        
        if self.num_tries == self.max_tries:
            self.num_tries = 0
            self.feedback.state = 'Navigation failed, please try again.'
            self.feedback.alert_type = "error"
            self.server.publish_feedback(self.feedback)
            self.server.set_aborted()
            return

        self.rotate_to_place()

        rospy.sleep(0.1)
        self.feedback.state = 'Scanning for marker...'
        self.feedback.alertType = "info"
        self.server.publish_feedback(self.feedback)
        head_scan_goal = HeadScanGoal()
        head_scan_goal.name = self.aruco_name
        self.head_scan_client.send_goal_and_wait(head_scan_goal)
        if not self.head_scan_client.get_result(): 
            self.feedback.state = 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
            self.feedback.alert_type = "error"
            self.server.publish_feedback(self.feedback)
            self.server.set_aborted()
            return
                
        try:
            new_map_to_aruco = self.tf2_buffer.lookup_transform('map', self.aruco_name, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Could not publish pose to tf")
            self.feedback.state = 'Navigation failed, please try again.'
            self.feedback.alert_type = "error"
            self.server.publish_feedback(self.feedback)
            self.server.set_aborted()
            return

        diff = self.calculate_diff(self.map_to_aruco, new_map_to_aruco)
        if (diff > 0.075):
            self.reconfigure_client.update_configuration({"controller_frequency": 10.0})
            self.map_to_aruco = new_map_to_aruco
            self.navigate_to_marker()
        elif (diff > 0.05):
            self.reconfigure_client.update_configuration({"controller_frequency": 5.0})
            self.map_to_aruco = new_map_to_aruco
            self.navigate_to_marker()
        elif (diff > 0.025):
            self.reconfigure_client.update_configuration({"controller_frequency": 1.5})
            self.map_to_aruco = new_map_to_aruco
            self.navigate_to_marker()

        self.rotate_to_place()
        self.feedback.state = 'Navigation succeeded!'
        self.feedback.alert_type = "success"
        self.server.publish_feedback(self.feedback)
        self.server.set_succeeded()
        
        return
    
if __name__ == '__main__':
    rospy.init_node('navigate_to_aruco_server')
    server = NavigateToArucoServer()
    rospy.spin()