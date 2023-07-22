#!/usr/bin/env python3
import os

import numpy as np
import pandas as pd
import rclpy
from ament_index_python.packages import get_package_share_directory
from libf1tenth.planning import Waypoints
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from libf1tenth.util.viz import to_waypoints_viz_msg


class Visualizewaypoints(Node):

    def __init__(self):
        super().__init__('waypoint_visualizer')
        self.get_logger().info("Waypoint Visualizer initialized")
        
        # declare parameter waypoint_path
        # inner is inner
        self.declare_parameter('waypoint_path', 'waypoints/levine_blocked_waypoints_optimal.csv')
        self.declare_parameter('waypoint_path_inner', 'waypoints/levine_blocked_waypoints_optimal.csv')
        self.declare_parameter('waypoint_path_outer', 'waypoints/levine_blocked_waypoints_optimal.csv')
        self.waypoint_path = self.get_parameter('waypoint_path').get_parameter_value().string_value
        self.waypoint_path_inner = self.get_parameter('waypoint_path_inner').get_parameter_value().string_value
        self.waypoint_path_outer = self.get_parameter('waypoint_path_outer').get_parameter_value().string_value
        self.waypoint_path = os.path.join(get_package_share_directory('f1tenth_bringup'), self.waypoint_path)
        self.waypoint_path_inner = os.path.join(get_package_share_directory('f1tenth_bringup'), self.waypoint_path_inner)
        self.waypoint_path_outer = os.path.join(get_package_share_directory('f1tenth_bringup'), self.waypoint_path_outer)
        
        self.waypoints = Waypoints.from_csv(self.waypoint_path).upsample(5)
        self.waypoints_inner = Waypoints.from_csv(self.waypoint_path_inner).upsample(5)
        self.waypoints_outer = Waypoints.from_csv(self.waypoint_path_outer).upsample(5)
        
        self.waypoint_publisher = self.create_publisher(MarkerArray, '/waypoints',1)
        self.waypoint_publisher_inner = self.create_publisher(MarkerArray, '/waypoints_inner',1)
        self.waypoint_publisher_outer = self.create_publisher(MarkerArray, '/waypoints_outer',1)
        
        self.pub_timer_ = self.create_timer(1., self.publish_waypoints)
        self.waypoint_publisher_real = self.create_publisher(Float32MultiArray, '/waypoints_real',10)
        self.waypoint_publisher_real_inner = self.create_publisher(Float32MultiArray, '/inner_waypoints_real',10)
        self.waypoint_publisher_real_outer = self.create_publisher(Float32MultiArray, '/outer_waypoints_real',10)
        
    def publish_waypoints(self):
        lane_waypoints = [self.waypoints, self.waypoints_inner, self.waypoints_outer]
        publishers = [self.waypoint_publisher, self.waypoint_publisher_inner, self.waypoint_publisher_outer]
        publishers_real = [self.waypoint_publisher_real, self.waypoint_publisher_real_inner, self.waypoint_publisher_real_outer]
        
        for i in range(len(lane_waypoints)):
            waypoints = lane_waypoints[i]
            waypoint_publisher = publishers[i]
            waypoint_publisher_real = publishers_real[i]
            waypoint_visualize_message = to_waypoints_viz_msg(waypoints, ns='global_waypoints')
                
            waypoint_message = Float32MultiArray()
            N = len(waypoints)
            waypoint_data = np.zeros((N,6))
            waypoint_data[:,0] = waypoints.x
            waypoint_data[:,1] = waypoints.y
            waypoint_data[:,2] = waypoints.velocity
            waypoint_data[:,3] = waypoints.yaw
            waypoint_data[:,4] = waypoints.curvature
            waypoint_data[:,5] = waypoints.gain
            waypoint_message.data = waypoint_data.flatten().tolist()

            waypoint_publisher.publish(waypoint_visualize_message)
            waypoint_publisher_real.publish(waypoint_message)

def main(args=None):
    rclpy.init(args=args)
    visualize_way = Visualizewaypoints()
    rclpy.spin(visualize_way)
    visualize_way.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()

