#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from apriltag_msgs.msg import AprilTagDetectionArray

class AprilTagRelativePose(Node):
    '''
        This node computes the relative pose of the drone (base_link) with respect
        to the AprilTag frame using TF.
    '''
    def __init__(self):
        super().__init__('apriltag_relative_pose')

        # ---------------- Parameters ----------------
        self.declare_parameter('frames.world', 'map')
        self.declare_parameter('frames.tag', 'tag36h11:0')
        self.declare_parameter('frames.drone', 'base_link')

        self.world_frame = self.get_parameter('frames.world').value
        self.tag_frame = self.get_parameter('frames.tag').value
        self.drone_frame = self.get_parameter('frames.drone').value

        self.get_logger().info(f"[FRAMES] world={self.world_frame}, drone={self.drone_frame}, tag={self.tag_frame}")

        # ---------------- State storage ----------------
        self.tag_visible = False
        self.last_tag_detection_time = self.get_clock().now()

        # ---------------- TF Setup ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Publisher ----------------
        self.rel_pose_pub = self.create_publisher(PoseStamped, '/landing/relative_pose_drone', 10)
        self.tag_visible_pub = self.create_publisher(Bool, '/landing/tag_visible_flag', 10)
        self.tag_map_pub = self.create_publisher(PoseStamped, '/landing/tag_pose_map', 10)
        
        # ---------------- Subscription ----------------
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_detections_callback, 10)

        # ---------------- Timer ----------------
        self.timer = self.create_timer(0.05, self.update)           # 20 Hz

        self.get_logger().info('AprilTag Relative Pose Node has been started.')

    
    # ---------------- Callbacks ----------------
    def tag_detections_callback(self, msg):
        '''
            Callback for AprilTag detections.
        '''
        if len(msg.detections) > 0:
            self.tag_visible = True
            self.last_tag_detection_time = self.get_clock().now()
        else:
            self.tag_visible = False

    
    def publish_tag_in_map(self):
        '''
            Publish the AprilTag pose in the MAP frame as a PoseStamped message.
        '''
        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, self.tag_frame, rclpy.time.Time())

            t = transform.transform.translation

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.world_frame

            msg.pose.position.x = t.x
            msg.pose.position.y = t.y
            msg.pose.position.z = t.z

            self.tag_map_pub.publish(msg)

        except Exception as e:
            pass                

    
    def update(self):
        '''
            Lookup the transform:
                apriltag --> base_link
            
            and publish it as a PoseStamped message.

            Transform from apriltag frame to base_link frame gives the pose of
            the drone relative to the AprilTag.
        '''
        if not self.tf_buffer.can_transform(self.tag_frame, self.drone_frame, rclpy.time.Time()):
            return
        
        # If no detections for more than 0.3 seconds, consider tag not visible/lost
        dt = (self.get_clock().now() - self.last_tag_detection_time).nanoseconds * 1e-9
        if dt > 0.3:
            self.tag_visible = False

        # Publish tag visibility
        tag_visible_msg = Bool()
        tag_visible_msg.data = self.tag_visible
        self.tag_visible_pub.publish(tag_visible_msg)
        
        try:
            transform = self.tf_buffer.lookup_transform(self.tag_frame, self.drone_frame, rclpy.time.Time())

        except Exception as e:
            self.get_logger().warn(f'Could not transform {self.tag_frame} to {self.drone_frame}: {e}')
            return
        
        # If tag not visible, do not publish
        if not self.tag_visible:
            return
        
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        
        # Convert TransformStamped to PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.tag_frame

        pose_msg.pose.position.x = -tx
        pose_msg.pose.position.y = -ty
        pose_msg.pose.position.z = tz

        pose_msg.pose.orientation = transform.transform.rotation

        # Publish the relative pose
        self.rel_pose_pub.publish(pose_msg)

        # Also publish the tag pose in map frame
        self.publish_tag_in_map()


def main():
    rclpy.init()
    node = AprilTagRelativePose()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()