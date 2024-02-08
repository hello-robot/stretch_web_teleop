#! /usr/bin/env python3

import rclpy
import tf2_ros
from threading import Event
# import dynamic_reconfigure.client
from math import fabs, sqrt
from geometry_msgs.msg import TransformStamped, Transform
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray
import numpy as np
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import PoseStamped
from stretch_teleop_interface_msgs.action import HeadScan, NavigateToAruco
from stretch_teleop_interface_msgs.srv import RelativePose
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import math

from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

class NavigateToArucoActionServer(Node):
    def __init__(self):
        super().__init__('navigate_to_aruco_server')
        self.action_done_event = Event()

        self.server = ActionServer(self, NavigateToAruco, 'navigate_to_aruco', self.execute_callback)
        self.relative_pose_service = self.create_service(RelativePose, 'get_relative_pose', self.get_relative_pose_callback)
        self.switch_to_position_mode_client = self.create_client(Trigger, 'switch_to_position_mode')
        self.switch_to_navigation_mode_client = self.create_client(Trigger, 'switch_to_navigation_mode')
        self.server.register_cancel_callback(self.cancel_goals)
        self.goal = NavigateToAruco.Goal()
        self.feedback = NavigateToAruco.Feedback()
        self.result = NavigateToAruco.Result()
        self.num_tries = 0
        self.max_tries = 5
        self.preempt = False
        # self.reconfigure_client = dynamic_reconfigure.client.Client("move_base")

        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer, self)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf2_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.head_scan_client = ActionClient(
            self,
            HeadScan,
            "head_scan",
        )
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.head_to_aruco_client = ActionClient(
            self,
            HeadScan,
            "head_to_aruco",
        )
    
    def cancel_goals(self):
        print('cancel goals')
        self.head_scan_client._cancel_goal_async()
        self.nav_to_pose_client._cancel_goal_async()
        # self.server.set_preempted()

    def preempt_callback(self):
        print('preempt callback')
        self.feedback.state = 'Aruco navigation cancelled!'
        self.feedback.alert_type = "error"
        
    def broadcast_tf(self, trans, name, ref):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ref
        t.child_frame_id = name
        t.transform = trans
        return t
    
    def get_relative_pose_callback(self, request, response):
        # import IPython; IPython.embed()
        self.aruco_name = request.name
        # self.action_done_event.clear()

        # self.feedback.state = 'Scanning for marker...'
        # self.feedback.alert_type = "info"
        # # self.server.publish_feedback(self.feedback)
        # self.switch_to_position_mode_client.call_async(Trigger.Request())

        # head_scan_goal = HeadScan.Goal()
        # head_scan_goal.name = self.aruco_name
        # self.head_scan_client.wait_for_server()
        # send_goal_future = self.head_scan_client.send_goal_async(head_scan_goal)
        # send_goal_future.add_done_callback(self.goal_response_callback)

        # # Wait for action to be done
        # self.action_done_event.wait()

        # if not self.send_goal_result:
        #     self.feedback.state = 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
        #     self.feedback.alert_type = "error"
        #     # self.server.publish_feedback(self.feedback)
        #     # self.server.set_aborted()
        #     return

        try:
            map_pose = self.tf2_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose_stamped =  PoseStamped()
            pose_stamped.header = map_pose.header
            pose_stamped.pose.position.x = map_pose.transform.translation.x
            pose_stamped.pose.position.y = map_pose.transform.translation.y
            pose_stamped.pose.position.z = map_pose.transform.translation.z
            pose_stamped.pose.orientation.x = map_pose.transform.rotation.x
            pose_stamped.pose.orientation.y = map_pose.transform.rotation.y
            pose_stamped.pose.orientation.z = map_pose.transform.rotation.z
            pose_stamped.pose.orientation.w = map_pose.transform.rotation.w

            trans = self.tf2_buffer.transform(pose_stamped, self.aruco_name, rclpy.duration.Duration(seconds=1.0))
        
            transform = Transform()
            transform.translation.x = trans.pose.position.x
            transform.translation.y = trans.pose.position.y
            transform.translation.z = trans.pose.position.z
            transform.rotation.x = trans.pose.orientation.x
            transform.rotation.y = trans.pose.orientation.y
            transform.rotation.z = trans.pose.orientation.z
            transform.rotation.w = trans.pose.orientation.w

            self.get_logger().info("Returning transform")
            response.transform = transform
            return response
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("Could not publish pose to tf")
            return

    def execute_callback(self, goal):
        # self.reconfigure_client.update_configuration({"controller_frequency": 20.0})
        print('callback')
        self.num_tries = 0
        self.goal = goal
        self.aruco_name = self.goal.request.name
        self.relative_pose = self.goal.request.pose

        self.action_done_event.clear()

        self.feedback.state = 'Scanning for marker...'
        self.feedback.alert_type = "info"
        # self.server.publish_feedback(self.feedback)
        self.switch_to_position_mode_client.call_async(Trigger.Request())

        head_scan_goal = HeadScan.Goal()
        head_scan_goal.name = self.aruco_name
        self.head_scan_client.wait_for_server()
        send_goal_future = self.head_scan_client.send_goal_async(head_scan_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Wait for action to be done
        self.action_done_event.wait()

        # if self.server.is_preempt_requested():
        #     self.preempt_callback()
        #     return 
        
        if not self.send_goal_result:
            self.feedback.state = 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
            self.feedback.alert_type = "error"
            # self.server.publish_feedback(self.feedback)
            # self.server.set_aborted()
            result = NavigateToAruco.Result()
            result.success = False
            self.goal.abort()
            return result

        self.marker = self.tf2_buffer.lookup_transform('base_link', self.aruco_name, self.get_clock().now(), timeout=rclpy.duration.Duration(seconds=5.0))
        self.map_to_aruco = self.tf2_buffer.lookup_transform('map', self.aruco_name, self.get_clock().now(), timeout=rclpy.duration.Duration(seconds=5.0))
        self.aruco_tf = self.broadcast_tf(self.map_to_aruco.transform, self.aruco_name, 'map')
        self.tf2_broadcaster.sendTransform(self.aruco_tf)
        self.get_logger().info("{} pose published to tf".format(self.aruco_name))

        return self.navigate_to_marker()
    
    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Signal that action is done
        try:
            self.send_goal_result = future.result().result.success
        except:
            self.send_goal_result = True # future.result().result.success
        self.action_done_event.set()

    def average_transforms(self, transforms):
        trans = transforms[0]
        trans.transform.translation.z = 0.0
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
        self.switch_to_navigation_mode_client.call_async(Trigger.Request())

        # if self.server.is_preempt_requested():
        #     self.preempt_callback()
        #     return
        
        self.num_tries += 1
        tran = self.broadcast_tf(self.relative_pose, 'relative_pose', self.aruco_name)
        self.tf2_static_broadcaster.sendTransform(tran)
        transforms = []
        self.get_logger().info("Broadcasting relative pose")

        self.feedback.state = "Navigating to marker..."
        self.feedback.alert_type = "info"
        # self.server.publish_feedback(self.feedback)

        while len(transforms) < 50:
            try:
                trans = self.tf2_buffer.lookup_transform('map', 'relative_pose', self.get_clock().now(), timeout=rclpy.duration.Duration(seconds=10.0))
                transforms.append(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().info("Could not publish pose to tf")
                self.feedback.state = 'Navigation failed, please try again.'
                self.feedback.alert_type = "error"
                # self.server.publish_feedback(self.feedback)
                # self.server.set_aborted()
                result = NavigateToAruco.Result()
                result.success = False
                self.goal.abort()
                return result

        print(self.average_transforms(transforms))
        self.relative_pose_tf = trans

        self.get_logger().info("Published relative pose")

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = self.relative_pose_tf.transform.translation.x
        goal_pose.pose.position.y = self.relative_pose_tf.transform.translation.y
        goal_pose.pose.position.z = self.relative_pose_tf.transform.translation.z
        goal_pose.pose.orientation.x = self.relative_pose_tf.transform.rotation.x
        goal_pose.pose.orientation.y = self.relative_pose_tf.transform.rotation.y
        goal_pose.pose.orientation.z = self.relative_pose_tf.transform.rotation.z
        goal_pose.pose.orientation.w = self.relative_pose_tf.transform.rotation.w

        action_goal = NavigateToPose.Goal()
        # action_goal.header.frame_id = "map"
        # action_goal.header.stamp = rospy.Time.now()
        action_goal.pose = goal_pose

        # head_scan_goal = HeadScanGoal()
        # head_scan_goal.name = self.aruco_name
        # self.head_to_aruco_client.send_goal(head_scan_goal)
        self.action_done_event.clear()
        
        self.switch_to_navigation_mode_client.call_async(Trigger.Request())
        send_goal_future = self.nav_to_pose_client.send_goal_async(action_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Wait for action to be done
        self.action_done_event.wait()

        # self.head_to_aruco_client.cancel_all_goals()

        # self.move_base_aruco_callback(send_goal_result)
        self.rotate_to_place()

        result = NavigateToAruco.Result()
        result.success = True
        self.goal.succeed()
        return result

    def rotate_to_place(self):
        goal_angles = euler_from_quaternion([self.relative_pose_tf.transform.rotation.x,
                                             self.relative_pose_tf.transform.rotation.y,
                                             self.relative_pose_tf.transform.rotation.z,
                                             self.relative_pose_tf.transform.rotation.w])
        goal_theta = goal_angles[2]
        goal_x = self.relative_pose_tf.transform.translation.x
        goal_y = self.relative_pose_tf.transform.translation.y

        diff = 0.5
        distance = 0.5 
        heading_error = 0.5
        while math.fabs(distance) > 0.05 or math.fabs(heading_error) > 0.025:
            if self.goal.is_cancel_requested: 
                self.cancel_goals()
                result = NavigateToAruco.Result()
                result.success = False
                self.goal.succeed()
                return result

            try:
                curr_pose = self.tf2_buffer.lookup_transform('map', 'base_link', self.get_clock().now(), timeout=rclpy.duration.Duration(seconds=5.0))
                curr_angles = euler_from_quaternion([curr_pose.transform.rotation.x,
                                                     curr_pose.transform.rotation.y,
                                                     curr_pose.transform.rotation.z,
                                                     curr_pose.transform.rotation.w])
                diff = goal_theta - curr_angles[2]

                x = curr_pose.transform.translation.x
                y = curr_pose.transform.translation.y
                curr_theta = curr_angles[2]
                delta_x = goal_x - x
                delta_y = goal_y - y
                goal_heading = math.atan2(delta_y, delta_x)
                distance = math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
                heading_error = goal_theta - curr_theta
                if heading_error > math.pi: heading_error = heading_error - (2 * math.pi)
                if heading_error < -1*math.pi: heading_error = heading_error + (2 * math.pi)

                if curr_theta > math.pi: curr_theta = curr_theta - (2 * math.pi)
                if curr_theta < -1*math.pi: curr_theta = curr_theta + (2 * math.pi)
                if goal_heading > math.pi: goal_heading = goal_heading - (2 * math.pi)
                if goal_heading < -1*math.pi: goal_heading = goal_heading + (2 * math.pi)

                if math.fabs(distance) > 0.05:
                    if math.fabs(goal_heading - curr_theta) > 0.05:
                        # print('distance heading: ', math.fabs(goal_heading - curr_theta))
                        self.cmd_vel.linear.x = 0.0
                        self.cmd_vel.angular.z = 2*(goal_heading - curr_theta)
                    else:
                        # print('distance: ', math.fabs(distance))
                        self.cmd_vel.linear.x = 5*distance
                        self.cmd_vel.angular.z = 0.0
                else:
                    print('heading: ', heading_error)
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.5*heading_error

                # self.cmd_vel.angular.z = 0.5*diff
                self.cmd_vel_pub.publish(self.cmd_vel)            
            except:
                print('except')
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)

        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)
        
    def move_base_aruco_callback(self, result):
        if self.server.is_preempt_requested():
            self.preempt_callback()
            return
        
        if self.num_tries == self.max_tries:
            self.num_tries = 0
            self.feedback.state = 'Navigation failed, please try again.'
            self.feedback.alert_type = "error"
            # self.server.publish_feedback(self.feedback)
            # self.server.set_aborted()
            return

        self.rotate_to_place()

        rclpy.sleep(0.1)
        self.feedback.state = 'Scanning for marker...'
        self.feedback.alertType = "info"
        # self.server.publish_feedback(self.feedback)
        head_scan_goal = HeadScan.Goal()
        head_scan_goal.name = self.aruco_name
        future = self.head_scan_client.send_goal_async(head_scan_goal)
        if not future.result().result: 
            self.feedback.state = 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
            self.feedback.alert_type = "error"
            # self.server.publish_feedback(self.feedback)
            # self.server.set_aborted()
            return
                
        try:
            new_map_to_aruco = self.tf2_buffer.lookup_transform('map', self.aruco_name, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("Could not publish pose to tf")
            self.feedback.state = 'Navigation failed, please try again.'
            self.feedback.alert_type = "error"
            # self.server.publish_feedback(self.feedback)
            # self.server.set_aborted()
            return

        diff = self.calculate_diff(self.map_to_aruco, new_map_to_aruco)
        if (diff > 0.075):
            # self.reconfigure_client.update_configuration({"controller_frequency": 10.0})
            self.map_to_aruco = new_map_to_aruco
            self.navigate_to_marker()
        elif (diff > 0.05):
            # self.reconfigure_client.update_configuration({"controller_frequency": 5.0})
            self.map_to_aruco = new_map_to_aruco
            self.navigate_to_marker()
        elif (diff > 0.025):
            # self.reconfigure_client.update_configuration({"controller_frequency": 1.5})
            self.map_to_aruco = new_map_to_aruco
            self.navigate_to_marker()

        self.rotate_to_place()
        self.feedback.state = 'Navigation succeeded!'
        self.feedback.alert_type = "success"
        # self.server.publish_feedback(self.feedback)
        # self.server.set_succeeded()
        
        return
    
if __name__ == '__main__':
    rclpy.init()
    node = NavigateToArucoActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()