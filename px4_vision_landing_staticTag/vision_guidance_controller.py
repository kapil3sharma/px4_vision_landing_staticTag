#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from px4_vision_landing_staticTag.utils.common import now_us, flu_to_frd
from px4_vision_landing_staticTag.utils.px4_commands import land_command

class VisionGuidanceController(Node):
    '''
        Vision-based Guidance controller for precision landing on a static AprilTag.

        - Uses KF-filtered AprilTag pose estimates in the map frame.
        - Published setpoints to PX4 for offboard control.
        - PX4 internal MPC handles control.
        - Activated only when mission state == VISION_ACTIVE.
    '''
    def __init__(self):
        super().__init__('vision_guidance_controller')

        # ---------------- Parameters ----------------
        self.declare_parameter('control_rate', 20.0)            # Hz
        self.declare_parameter('descent_rate', 0.25)            # m/s
        self.declare_parameter('min_landing_altitude', 0.15)    # m
        self.declare_parameter('xy_tolerance', 0.15)            # m

        self.control_rate = self.get_parameter('control_rate').value
        self.descent_rate = self.get_parameter('descent_rate').value
        self.min_landing_altitude = self.get_parameter('min_landing_altitude').value
        self.xy_tolerance = self.get_parameter('xy_tolerance').value

        # ---------------- State storage ----------------
        self.mission_state = None               # Current mission state
        self.tag_pose = None                    # AprilTag pose (x, y, z) in 'map' frame (KF filtered)
        self.tag_visible = False                # AprilTag visibility flag

        self.drone_xy_enu = None                # Drone (x, y) position in 'map' frame
        self.drone_alt = None                   # Drone altitude in 'map' frame
        self.current_setpoint_z = None          # Current setpoint altitude

        self.state = 'WAIT_FOR_TAG'             # Controller state

        # ---------------- QoS Profile ----------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Subscriptions ----------------
        self.create_subscription(String, '/mission/state', self.mission_state_callback, qos)
        # self.create_subscription(PoseStamped, '/landing/tag_pose_map_kf_filtered', self.tag_pose_callback, qos)
        self.create_subscription(PoseStamped, '/landing/tag_pose_map', self.tag_pose_callback, qos)
        self.create_subscription(Bool, '/landing/tag_visible_flag', self.tag_visible_callback, qos)
        self.create_subscription(PoseStamped, '/debug/drone_pose_enu', self.drone_pose_enu_callback, qos)

        # ---------------- Publishers ----------------
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ---------------- Timer ----------------
        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("[VISION_GUIDANCE_CONTROLLER] Vision Guidance Controller Node has been started.")


    # ---------------- Callbacks ----------------
    def mission_state_callback(self, msg: String):
        '''
            Callback for mission state updates.
        '''
        self.mission_state = msg.data

    def tag_pose_callback(self, msg: PoseStamped):
        '''
            Callback for KF-filtered AprilTag pose.
        '''
        self.tag_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def tag_visible_callback(self, msg: Bool):
        '''
            Callback for AprilTag visibility flag.
        '''
        self.tag_visible = msg.data

    def drone_pose_enu_callback(self, msg: PoseStamped):
        '''
            Callback for drone pose in ENU frame (for debugging).
        '''
        self.drone_xy_enu = np.array([msg.pose.position.x, msg.pose.position.y])
        self.drone_alt = msg.pose.position.z


    # ---------------- Utilities ----------------
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

        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        '''
            Publish a position setpoint in ENU frame.
        '''
        sp = TrajectorySetpoint()
        sp.timestamp = now_us(self)
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float('nan')
        self.setpoint_pub.publish(sp)

    def land(self):
        '''
            Send command to land the drone.
        '''
        self.command_pub.publish(land_command(now_us(self)))
        self.get_logger().info("[OFFBOARD] Land command sent")


    # ---------------- Main Control Loop ----------------
    def control_loop(self):
        '''
            Main control loop for vision-based guidance.
        '''
        if self.mission_state != 'VISION_ACTIVE':
            self.get_logger().info(f"self.mission_state: {self.mission_state}", throttle_duration_sec=2.0)
            return
        
        if self.drone_alt is None or self.tag_pose is None:
            self.get_logger().info("[VISION_GUIDANCE_CONTROLLER] Waiting for valid drone altitude and tag pose...", throttle_duration_sec=2.0)
            return
        
        if self.mission_state == 'VISION_ACTIVE' and self.state == 'WAIT_FOR_TAG':
            self.get_logger().info("[VISION] Vision controller ACTIVE")

        self.publish_offboard_mode()

        if self.state == 'WAIT_FOR_TAG':
            if self.tag_visible:
                self.current_setpoint_z = self.drone_alt
                self.state = 'ALIGN'
                self.get_logger().info("[VISION_GUIDANCE_CONTROLLER] Tag detected. Transitioning to ALIGN state.")

        elif self.state == 'ALIGN':
            # Convert target position from ROS FLU frame (vision / TF) to PX4 FRD frame required for offboard position control
            x_cmd, y_cmd, z_cmd = flu_to_frd(self.tag_pose[0], self.tag_pose[1], self.current_setpoint_z)
            
            self.publish_position_setpoint(x_cmd, y_cmd, z_cmd)
            
            xy_error = np.linalg.norm(self.tag_pose[0:2] - self.drone_xy_enu)

            if xy_error < self.xy_tolerance:
                self.state = 'DESCEND'
                # self.get_logger().info("[VISION_GUIDANCE_CONTROLLER] Aligned over tag. Transitioning to DESCEND state.")
                return

        elif self.state == 'DESCEND':
            self.current_setpoint_z = max(self.current_setpoint_z - self.descent_rate / self.control_rate, self.min_landing_altitude)

            # Convert target position from ROS FLU frame (vision / TF) to PX4 FRD frame required for offboard position control
            x_cmd, y_cmd, z_cmd = flu_to_frd(self.tag_pose[0], self.tag_pose[1], self.current_setpoint_z)

            self.publish_position_setpoint(x_cmd, y_cmd, z_cmd)

            if self.current_setpoint_z <= self.min_landing_altitude:
                self.state = 'LAND'
                self.get_logger().info('[VISION] LAND')

        elif self.state == 'LAND':
            self.land()
            self.state = 'DONE'


# ---------------- Main Entry Point ----------------
def main():
    rclpy.init()
    node = VisionGuidanceController()

    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()