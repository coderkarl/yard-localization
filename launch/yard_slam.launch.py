from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number
from math import pi

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    yard_slam_node = Node(
        package="yard_slam",
        executable="yard_slam_node",
        output='screen',
        emulate_tty='True',
        parameters=[{'plan_rate_hz': 10.0},
                    {'min_range': 0.2},
                    {'max_range': 20.0},
                    {'topic_pointcloud_in': 'none'},
                    {'topic_pointcloud_out': 'keypoints'},
                    {'init_map_x': -16.0},
                    {'init_map_y': 20.0},
                    {'init_yaw_deg': -35.0},
                    {'init_x_sigma': 1.0},
                    {'init_y_sigma': 1.0},
                    {'init_yaw_sigma_deg': 5.0},
                    {'enc_sigma': 0.05}, #0.1
                    {'yaw_rate_sigma_rad': 0.01}, #0.03
                    {'meas_sigma': 1.0},
                    {'x_trees':          [8.5, 9.6, 17.7, 25.2]}, # Front yard
                    {'y_trees':          [43.5, 36.5, 36.5, 36.5]}, # Front yard
                    {'tree_dist_tresh': 2.0},
                    {'meas_reset_time_sec': 1.0},
                    {'use_sim_time': True}
        ]
        #remappings=[('cmd_vel', 'ignore_cmd_vel')]
    )
    
    static_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        arguments=['0.0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    ld.add_action(yard_slam_node)
    
    return ld
