#!/usr/bin/env python3
import sys
import traceback

import numpy as np
import time
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from visualization_msgs.msg import Marker

from libf1tenth.controllers import PDSpeedController, LateralLQRController, PurePursuitController
from libf1tenth.filter import MovingAverageFilter
from libf1tenth.planning import Pose
from libf1tenth.util.viz import to_waypoint_target_viz_msg


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    
    tuning table:
    
    10m/s+: K=0.6, lookahead=0.6
    4.7m/s: K=0.45, lookahead=0.6
    
    10m/s: K=0.34, lookahead=0.65 / 0.44+1.1
    
    ###### stanley sim ######
    3.0m/s: K=1.2, lookahead=0.25
    4.714m/s: K=1.2, lookahead=0.45
    6.429m/s: K=1.2, lookahead=0.9
    8.143m/s: K=1.2, lookahead=0.9
    9.857m/s: K=1.2, lookahead=0.9
        
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.get_logger().info("Pure Pursuit Node initialized")
        
        # global parameters
        self.parameter_client = self.create_client(GetParameters, 
                                                   '/global_parameter_server/get_parameters')
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = GetParameters.Request()
        request.names = ['use_sim_time', 'pose_topic', 'scan_topic', 'drive_topic']
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        params = future.result().values
        self.get_logger().info("global parameter service call succeeded")
        
        self.use_sim_time = params[0].bool_value
        self.pose_topic = params[1].string_value
        self.scan_topic = params[2].string_value
        self.drive_topic = params[3].string_value
        self.laser_frame_id = 'ego_racecar/laser' if self.use_sim_time else 'laser'
        
        # subscribers
        self.waypoint_sub = self.create_subscription(Float32MultiArray, '/waypoints_real', self.waypoint_callback, 10)
        self.waypoints = None
        self.pose_sub = self.create_subscription(Odometry, self.pose_topic, self.pose_callback, 10)
        
        # publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.waypoint_target_pub = self.create_publisher(Marker, '/waypoint_target', 10)
        
        # controllers
        #self.lateral_controller = StanleyController(K=1.2, Kd=0.1, wheelbase=0.58)
        self.lateral_controller = PurePursuitController(1.5) # Q [0.15,0.005,0.007,0.0025],
        # self.speed_controller = PDSpeedController(1.0, 0.1, buffer_size=10)

        self.prev_angle = 0.0
        self.avg_filter = MovingAverageFilter(10)
        self.avg_filter.update(self.prev_angle)
        
    def publish_drive(self, velocity, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)
        
    def publish_waypoint_target(self, waypoint_to_track):
        marker = to_waypoint_target_viz_msg(waypoint_to_track)
        self.waypoint_target_pub.publish(marker)
    
    def waypoint_callback(self, waypoint_msg):
        waypoint_data = waypoint_msg.data
        self.waypoints = np.array(waypoint_data).reshape((-1,5))

    def pose_callback(self, pose_msg):
        
        if self.waypoints is None:
            return
        
        pose = Pose.from_msg(pose_msg)
        t0 = time.time()
        angle, waypoint_to_track, vel = self.lateral_controller.get_steering_angle(pose, self.waypoints)
        t1 = time.time()
        self.get_logger().info("pure pursuit freq: {}".format(1.0/(t1-t0)))
        
        self.publish_waypoint_target(waypoint_to_track)

        target_velocity = vel
        # ctrl_velocity = self.speed_controller.control(pose.velocity, angle, target_velocity)
        
        self.publish_drive(min(target_velocity, 3.5), angle)
        self.avg_filter.update(angle)
        
def main(args=None):
    rclpy.init(args=args)
    pp_node = PurePursuit()
    rclpy.spin(pp_node)

    pp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
