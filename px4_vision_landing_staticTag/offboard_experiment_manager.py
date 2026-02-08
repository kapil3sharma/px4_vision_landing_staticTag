#!/usr/bin/env python3

import px4_msgs
import rclpy
from rclpy.node import Node
import sys

from std_msgs.msg import String, Bool
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from px4_vision_landing_staticTag.utils.trajectories import spiral_trajectory
from px4_vision_landing_staticTag.utils.common import now_us, yaw_to_quaternion, reached_position
from px4_vision_landing_staticTag.utils.px4_commands import arm_command, offboard_mode_command, land_command


class OffboardExperimentManager(Node):
    '''
        Offboard Experiment Manager for:
        - TAKEOFF
        - SEARCH AprilTag
        - Handover to Vision Guidance Controller when tag is detected 

    '''
    def __init__(self):
        super().__init__('offboard_experiment_manager')

        # ----------------------- Parameters ------------------------
        self.declare_parameter('flight_speed', 0.3)
        self.declare_parameter('target_altitude', 3.0)
        self.declare_parameter('waypoint_threshold', 0.25)

        self.flight_speed = self.get_parameter('flight_speed').value
        self.target_altitude = self.get_parameter('target_altitude').get_parameter_value().double_value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').get_parameter_value().double_value
        
        self.takeoff_z = -self.target_altitude          # PX4 NED to ENU conversion of altitude

        # ----------------------- State -------------------------
        self.state = 'TAKEOFF'
        self.position = None
        self.home_xy = None
        self.yaw = 0.0
        self.trajectory = []
        self.waypoint_index = 0

        self.tag_visible = False
        self.offboard_counter = 0
        self.offboard_enabled = False

        # ----------------------- QoS Profile ------------------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ----------------------- Subscriptions ------------------------
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos)
        self.create_subscription(Bool, '/landing/tag_visible_flag', self.tag_visible_callback, qos)
        self.create_subscription(px4_msgs.msg.VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos)

        # ----------------------- Publications -------------------------
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)     
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        
        self.marker_pub = self.create_publisher(Marker, '/drone_marker', qos_reliable)
        self.drone_pose_enu_pub = self.create_publisher(PoseStamped, '/debug/drone_pose_enu', qos)
        self.state_pub = self.create_publisher(String, '/mission/state', qos_reliable)

        # ----------------------- TF Broadcaster ------------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # Frame ids
        self.declare_parameter('frames.world', 'map')
        self.declare_parameter('frames.drone', 'base_link')

        self.world_frame = self.get_parameter('frames.world').value
        self.drone_frame = self.get_parameter('frames.drone').value

        self.get_logger().info(f"[OFFBOARD] Using world frame: {self.world_frame}")
        self.get_logger().info(f"[OFFBOARD] Using drone frame: {self.drone_frame}")

        # ----------------------- Timer ------------------------
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("[OFFBOARD] Offboard Experiment Manager Node has been started:")
        self.get_logger().info(f"    - Flight Speed: {self.flight_speed} m/s")
        self.get_logger().info(f"    - Target Altitude: {self.target_altitude} m")


    # -------------------------- Utilities --------------------------
    def set_state(self, new_state):
        '''
            Set the current mission state and publish it.
        '''
        if self.state != new_state:
            self.state = new_state
            state_msg = String()
            state_msg.data = self.state
            self.state_pub.publish(state_msg)
            self.get_logger().info(f"[MISSION] State changed to: {self.state}")
    

    # ----------------------- Callbacks ------------------------
    def local_position_callback(self, msg):
        '''
            Callback for drone local position updates in ENU frame.
        '''
        if not msg.xy_valid or not msg.z_valid:
            return

        self.position = [msg.x, msg.y, msg.z]
        self.yaw = msg.heading

        if self.home_xy is None:
            self.home_xy = [msg.x, msg.y]

        # -------- TF map -> base_link (ENU) --------
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame  
        t.child_frame_id = self.drone_frame

        t.transform.translation.x = msg.x
        t.transform.translation.y = -msg.y
        t.transform.translation.z = -msg.z

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # Publish drone pose in ENU frame from NED frame data
        drone_pose_enu = PoseStamped()
        drone_pose_enu.header = t.header

        drone_pose_enu.pose.position.x = t.transform.translation.x
        drone_pose_enu.pose.position.y = t.transform.translation.y
        drone_pose_enu.pose.position.z = t.transform.translation.z

        drone_pose_enu.pose.orientation.x = t.transform.rotation.x
        drone_pose_enu.pose.orientation.y = t.transform.rotation.y
        drone_pose_enu.pose.orientation.z = t.transform.rotation.z
        drone_pose_enu.pose.orientation.w = t.transform.rotation.w

        self.drone_pose_enu_pub.publish(drone_pose_enu)

    def tag_visible_callback(self, msg: Bool):
        '''
            Callback for AprilTag visibility flag.
        '''
        self.tag_visible = msg.data

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.get_logger().info(f"[STATUS] Nav State: {self.nav_state}, Armed: {self.arming_state == 4}", 
                            throttle_duration_sec=1.0)


    # ----------------------- PX4 Control ------------------------
    def publish_offboard_mode(self):
        '''
            Publish Offboard Control Mode message to enable position control.
        '''
        msg = OffboardControlMode()
        msg.timestamp = now_us(self)

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw=None):
        '''
            Publish a position setpoint in ENU frame.
        '''
        sp = TrajectorySetpoint()
        sp.timestamp = now_us(self)
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float('nan') if yaw is None else float(yaw)

        self.get_logger().info(f"[OFFBOARD] Publishing setpoint: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={sp.yaw}", throttle_duration_sec=1.0)
        self.setpoint_pub.publish(sp)

    def arm(self):
        '''
            Send command to arm the drone.
        '''
        self.command_pub.publish(arm_command(now_us(self)))
        self.get_logger().info("[OFFBOARD] Arm sent", throttle_duration_sec=1.0)

    def set_offboard(self):
        '''
            Send command to set offboard mode.
        '''
        self.command_pub.publish(offboard_mode_command(now_us(self)))
        self.get_logger().info("[OFFBOARD] Offboard mode requested", throttle_duration_sec=1.0)

    def land(self):
        '''
            Send command to land the drone.
        '''
        self.command_pub.publish(land_command(now_us(self)))
        self.get_logger().info("[OFFBOARD] Land command sent", throttle_duration_sec=1.0)


    # ----------------------- Visualization ------------------------
    def publish_marker(self):
        '''
            Publish a visualization marker representing the drone position in world frame.
        '''
        m = Marker()
        # Arrow attached to drone body for visualization
        m.header.frame_id = self.drone_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'drone'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # Position at origin of base_link
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0

        # Identity rotation (inherits base_link orientation)
        m.pose.orientation.w = 1.0      

        m.scale.x = 0.7
        m.scale.y = 0.15
        m.scale.z = 0.15

        m.color.b = 1.0
        m.color.a = 1.0

        self.marker_pub.publish(m)
    

    # ----------------------- Main Control Loop ------------------------
    def control_loop(self):
        '''
            Main control loop for offboard experiment management.
        '''
        self.publish_marker()                   # Publish drone marker

        if self.position is None or self.state == 'VISION_ACTIVE':
            return

        self.publish_offboard_mode()            # Continue publishing offboard mode

        # PX4 Offboard Initialization
        if self.offboard_counter < 20:
            self.get_logger().info(f"[OFFBOARD] Initializing Offboard... Publishing setpoint {*self.home_xy, self.takeoff_z}", throttle_duration_sec=1.0)
            self.publish_position_setpoint(*self.home_xy, self.takeoff_z, yaw=None)
            self.offboard_counter += 1
            return

        if not self.offboard_enabled:
            self.set_offboard()
            self.arm()
            self.offboard_enabled = True
            self.get_logger().info(f"[OFFBOARD] Taking off to {self.target_altitude:.1f} m", throttle_duration_sec=1.0)
            return
        
        # ------------------------ TAKEOFF ------------------------
        if self.state == 'TAKEOFF':
            self.publish_position_setpoint(*self.home_xy, self.takeoff_z, yaw=None)

            current_altitude = -self.position[2]
            if abs(current_altitude - self.target_altitude) < 0.3:
                # Generate trajectory after takeoff for seaching the AprilTag
                self.trajectory = spiral_trajectory(self.home_xy[0], self.home_xy[1], self.takeoff_z, radius=0.5, flight_speed=self.flight_speed)

                self.set_state('SEARCH')
                self.waypoint_index = 0
                self.get_logger().info(f"[MISSION] Starting search with {len(self.trajectory)} waypoints.")

        # ------------------------ SEARCH ------------------------
        elif self.state == 'SEARCH':
            if self.tag_visible:
                self.set_state('VISION_ACTIVE')
                self.get_logger().info("[MISSION] AprilTag detected! Handover to Vision Guidance Controller.")
                return

            waypoint = self.trajectory[self.waypoint_index]
            self.publish_position_setpoint(waypoint[0], waypoint[1], waypoint[2])

            if reached_position(self.position[:2], waypoint[:2], self.waypoint_threshold):
                self.waypoint_index = (self.waypoint_index + 1) % len(self.trajectory)       


# ----------------------------- Main function -----------------------------
def main():
    rclpy.init()
    node = OffboardExperimentManager()
    
    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
