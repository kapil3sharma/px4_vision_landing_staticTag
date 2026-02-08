import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

config_dir = os.path.join(get_package_share_directory('px4_vision_landing_staticTag'), 'config')
frames_yaml = os.path.join(config_dir, 'frames.yaml')

with open(frames_yaml, 'r') as f:
    frames_config = yaml.safe_load(f)

# Navigate the ROS 2 parameter structure
frames_params = frames_config['/**']['ros__parameters']['frames']

drone_frame = frames_params['drone']
camera_frame = frames_params['camera']

def generate_launch_description():

    # Static TF: Drone (base_link) -> Camera
    # Rotations align ROS (Z-up) to Camera Optical (Z-forward)
    static_camera_tf = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='camera_tf',
                                arguments=[
                                            '0', '0', '0.10',
                                            '1.57079', '3.14159', '0',
                                            drone_frame,
                                            camera_frame        
                                        ],
                                parameters=[{'use_sim_time': True}],                        
                            )
    
    # Arguments
    trajectory_arg = DeclareLaunchArgument(
                                            name='trajectory_type',
                                            default_value='circle',
                                            description='Trajectory type: square, spiral, circle'
                                        )
    
    speed_arg = DeclareLaunchArgument(
                                        name='flight_speed',
                                        default_value='0.25',
                                        description='Drone speed scaling factor'
                                    )

    altitude_arg = DeclareLaunchArgument(
                                            name='target_altitude',
                                            default_value='3.0',
                                            description='Takeoff and flight altitude in meters'
                                        )
    
    waypoint_threshold_arg = DeclareLaunchArgument(
                                                    name='waypoint_threshold',
                                                    default_value='0.20',
                                                    description='Distance threshold to consider waypoint reached (meters) - x and y only'
                                                )

    # trajectory execution + experiment logic node
    offboard_experiment_manager_node = Node(
                                            package='px4_vision_landing_staticTag',
                                            executable='offboard_experiment_manager',
                                            name='offboard_experiment_manager_node',
                                            output='screen',
                                            parameters=[
                                                            frames_yaml,
                                                            {'use_sim_time': True},
                                                            {'trajectory_type': LaunchConfiguration('trajectory_type')},
                                                            {'flight_speed': LaunchConfiguration('flight_speed')},
                                                            {'target_altitude': LaunchConfiguration('target_altitude')},
                                                            {'waypoint_threshold': LaunchConfiguration('waypoint_threshold')},
                                                        ],
                                        )
    
    vision_guidance_controller_node = Node(
                                            package='px4_vision_landing_staticTag',
                                            executable='vision_guidance_controller',
                                            name='vision_guidance_controller_node',
                                            output='screen',
                                            parameters=[{'use_sim_time': True}],
                                        )
    
    # Event handler to shut down ROS2 after experiment manager node exits
    # This ensures all other nodes are also terminated
    shutdown_handler = RegisterEventHandler(
                                                event_handler=OnProcessExit(
                                                    target_action=offboard_experiment_manager_node,
                                                    on_exit=[EmitEvent(event=Shutdown())],
                                                )
                                            )
                
    # AprilTag pipeline node
    apriltag_pipeline_launch = IncludeLaunchDescription(
                                        PythonLaunchDescriptionSource(
                                            os.path.join(
                                                get_package_share_directory('px4_vision_landing_staticTag'),
                                                'launch',
                                                'apriltag_pipeline.launch.py'
                                            )
                                        )
                                    )
    
    # Perception + TF + relative pose of drone to AprilTag node
    apriltag_relative_pose_node = Node(
                                        package='px4_vision_landing_staticTag',
                                        executable='apriltag_relative_pose',
                                        name='apriltag_relative_pose_node',
                                        output='screen',
                                        parameters=[
                                                        {'use_sim_time': True},
                                                        frames_yaml,
                                                    ],
                                    )
    
    tag_rviz_markers_node = Node(
                                    package='px4_vision_landing_staticTag',
                                    executable='tag_rviz_markers',
                                    name='tag_rviz_markers_node',
                                    output='screen',
                                    parameters=[
                                                    {'use_sim_time': True},
                                                    frames_yaml,
                                                ],
                                )
    
    # Kalman Filter for AprilTag pose node
    tag_pose_kf_node = Node(
                            package='px4_vision_landing_staticTag',
                            executable='tag_pose_kf',
                            name='tag_pose_kf_node',
                            output='screen',
                            parameters=[
                                            {'use_sim_time': True},
                                            {'kf_rate': 20.0},
                                            frames_yaml,
                                        ],
                        )
    
    # Data logger for TF state + CSV output node
    tf_state_logger_node = Node(
                            package='px4_vision_landing_staticTag',
                            executable='tf_state_logger',
                            name='tf_state_logger',
                            output='screen',
                            parameters=[
                                {'use_sim_time': True},
                                {'log_dir': '/home/priyam22/px4_ros2_ws/src/px4_vision_landing_staticTag/px4_logs'}
                            ],
                        )
    
    
    return LaunchDescription([
                                # Launch Arguments
                                trajectory_arg,
                                speed_arg,
                                altitude_arg,
                                waypoint_threshold_arg,
                                
                                # TF Nodes
                                static_camera_tf,

                                # Camera pipeline Nodes
                                apriltag_pipeline_launch,
                                apriltag_relative_pose_node,

                                # Kalman Filter Node
                                tag_pose_kf_node,

                                offboard_experiment_manager_node,
                                vision_guidance_controller_node,
                                
                                # Debugging Nodes
                                tf_state_logger_node,
                                tag_rviz_markers_node,

                                # Event Handler
                                shutdown_handler,
                            ])