#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import csv
from datetime import datetime
import os
import math

from px4_vision_landing_staticTag.utils.common import now_sec


class TFStateLogger(Node):
    '''
        Debug + CSV logger node
    '''
    def __init__(self):
        super().__init__('tf_state_logger')

        # ---------------- Parameters ----------------
        self.log_rate_hz = 10.0                  # Log every 0.1 second
        
        # ---------------- State storage ----------------
        self.drone_ned_pose = None              # (x, y, z) NED in 'map' frame
        self.drone_enu_pose = None              # (x, y, z) ENU in 'map' frame
        self.tag_pose = None                    # (x, y, z) in apriltag frame
        self.tag_map_pose = None                # Apriltag pose (x, y, z) in 'map' frame
        self.tag_visible = False                # AprilTag visibility flag
        self.tag_map_kf_filtered_pose = None    # AprilTag pose (x, y, z) in 'map' frame (KF filtered)

        # ---------------- QoS Profile ----------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---------------- Subscriptions ----------------
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.drone_ned_pose_callback, qos)
        self.create_subscription(PoseStamped, '/debug/drone_pose_enu', self.drone_enu_pose_callback, qos)
        self.create_subscription(PoseStamped, '/landing/relative_pose_drone', self.tag_pose_callback, qos)
        self.create_subscription(PoseStamped, '/landing/tag_pose_map', self.tag_map_pose_callback, qos)
        self.create_subscription(Bool, '/landing/tag_visible_flag', self.tag_visible_callback, qos)
        self.create_subscription(PoseStamped, '/landing/tag_pose_map_kf_filtered', self.tag_map_kf_filtered_callback, qos)

        # --------------------- CSV Logger Setup ---------------------
        self.declare_parameter('log_dir', '~/px4_logs')
        log_dir = self.get_parameter('log_dir').value

        os.makedirs(log_dir, exist_ok=True)

        # filename = datetime.now().strftime('tf_state_log_%m_%d_%H_%M.csv')
        filename = 'px4_state_raw_tag_kf_filtered_2.csv'
        # filename = 'px4_state_tag_kf_filtered.csv'
        self.log_filepath = os.path.join(log_dir, filename)

        self.csv_file = open(self.log_filepath, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write CSV header
        self.csv_writer.writerow([
                                    'time',
                                    'drone_ned_x', 'drone_ned_y', 'drone_ned_z',
                                    'drone_enu_x', 'drone_enu_y', 'drone_enu_z',
                                    
                                    'rel_x', 'rel_y', 'rel_z',

                                    'tag_visible',

                                    'tag_map_x', 'tag_map_y', 'tag_map_z',

                                    'tag_map_kf_x', 'tag_map_kf_y', 'tag_map_kf_z'
                                ])

        # ---------------- Timer ----------------
        self.timer = self.create_timer(1.0 / self.log_rate_hz, self.log_state)

        self.get_logger().info('[TF STATE LOGGER] Started (logging at %.1f Hz)' % self.log_rate_hz)
        self.get_logger().info(f'[TF STATE LOGGER] Writing logs to: {self.log_filepath}')


    # --------------------------- Callbacks ---------------------------
    def drone_ned_pose_callback(self, msg: VehicleLocalPosition):
        '''
            Callback to store the vehicle position in NED 'map' frame.
        '''
        self.drone_ned_valid = msg.xy_valid and msg.z_valid
        if msg.xy_valid and msg.z_valid:
            self.drone_ned_pose = (msg.x, msg.y, msg.z)

    def drone_enu_pose_callback(self, msg: PoseStamped):
        '''
            Callback to store the vehicle position in ENU 'map' frame.
        '''
        self.drone_enu_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def tag_pose_callback(self, msg: PoseStamped):
        '''
            Callback to store the vehicle position in 'apriltag' frame.
        '''
        self.tag_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def tag_map_pose_callback(self, msg: PoseStamped):
        '''
            Callback to store the AprilTag position in 'map' frame.
        '''
        self.tag_map_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def tag_visible_callback(self, msg: Bool):
        '''
            Callback to store the tag visibility flag.
        '''
        self.tag_visible = msg.data

    def tag_map_kf_filtered_callback(self, msg: PoseStamped):
        '''
            Callback to store the AprilTag KF filtered position in 'map' frame.
        '''
        self.tag_map_kf_filtered_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


    # --------------------------- Logger ---------------------------
    def log_state(self):
        '''
            Log the drone and tag state to CSV and console.
        '''
        # Current time (seconds)
        t = now_sec(self)

        nan3 = (math.nan, math.nan, math.nan)

        # Write to CSV
        row = [
                t,
                *(self.drone_ned_pose if self.drone_ned_pose else nan3),
                *(self.drone_enu_pose if self.drone_enu_pose else nan3),
                *(self.tag_pose if self.tag_pose else nan3),
                
                int(self.tag_visible),
                *(self.tag_map_pose if self.tag_map_pose else nan3),
                *(self.tag_map_kf_filtered_pose if self.tag_map_kf_filtered_pose else nan3)
            ]
        
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        # ------------------------------ Console Log ------------------------------
        # self.get_logger().info('------------------- TF CHECK -------------------')

        # self.get_logger().info(
        #     f'  NED available: {"YES" if self.drone_ned_pose else "NO"} | '
        #     f'ENU available: {"YES" if self.drone_enu_pose else "NO"} | '
        #     f'Tag available: {"YES" if self.tag_pose else "NO"} | '
        #     f'Tag in MAP available: {"YES" if self.tag_map_pose else "NO"} | '
        #     f'Tag KF filtered in MAP available: {"YES" if self.tag_map_kf_filtered_pose else "NO"} | '
        #     f'Tag visible: {"YES" if self.tag_visible else "NO"}'
        # )

        # self.get_logger().info('------------------- TF STATE -------------------')

        # Map NED frame pose
        if self.drone_ned_pose is not None:
            x, y, z = self.drone_ned_pose
        else:
            self.get_logger().info('[MAP FRAME] Drone Position (NED): NOT AVAILABLE', throttle_duration_sec=1.0)

        # Map ENU frame pose
        if self.drone_enu_pose is not None:
            x, y, z = self.drone_enu_pose
        else:
            self.get_logger().info('[MAP FRAME] Drone Position (ENU): NOT AVAILABLE', throttle_duration_sec=1.0)

        # AprilTag frame pose
        if self.tag_pose is not None:
            x, y, z = self.tag_pose
        else:
            self.get_logger().info('[APRILTAG FRAME] Drone Position: NOT AVAILABLE', throttle_duration_sec=1.0)

        # Tag pose in MAP frame
        if self.tag_map_pose is not None:
            x, y, z = self.tag_map_pose
        else:
            self.get_logger().info('[MAP FRAME] AprilTag Position: NOT AVAILABLE', throttle_duration_sec=1.0)

        # Tag KF filtered pose in MAP frame
        if self.tag_map_kf_filtered_pose is not None:
            x, y, z = self.tag_map_kf_filtered_pose
        else:
            self.get_logger().info('[MAP FRAME] AprilTag KF Filtered Position: NOT AVAILABLE', throttle_duration_sec=1.0)

        # self.get_logger().info('-----------------------------------------------')

    def destroy_node(self):
        '''
            Cleanup on node destruction.
        '''
        print('[TF STATE LOGGER] Closing CSV file.')
        self.csv_file.close()
        super().destroy_node()

# --------------------------- Main ---------------------------
def main():
    rclpy.init()
    node = TFStateLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()