#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
# from tf_transformations.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import numpy as np

from time import gmtime, strftime

home = os.path.expanduser('~')
if not os.path.exists(os.path.join(home, 'waypoints')):
    os.makedirs(os.path.join(home, 'waypoints'))
file = open(os.path.join(home, 'waypoints/waypoints_%s.txt' % strftime("%Y-%m-%d_%H-%M-%S", gmtime())), 'w')

class Logwaypoints(Node):

    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # TODO: create ROS subscribers and publishers.
        odom_topic = '/pf/pose/odom'
        odom_sub = self.create_subscription(Odometry, odom_topic, self.save_waypoint, 10)


    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x, 
                        data.pose.pose.orientation.y, 
                        data.pose.pose.orientation.z, 
                        data.pose.pose.orientation.w])
        rot_obj = R.from_quat(quaternion)
        euler = rot_obj.as_euler('xyz')
        speed = np.linalg.norm(np.array([data.twist.twist.linear.x, 
                            data.twist.twist.linear.y, 
                            data.twist.twist.linear.z]),2)
        # if data.twist.twist.linear.x>0.:
        #     print data.twist.twist.linear.x
        
        print(data.pose.pose.position.x, data.pose.pose.position.y, euler[2], speed)

        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                 data.pose.pose.position.y,
                                 euler[2],
                                 speed))

def main(args=None):
    rclpy.init(args=args)
    logway = Logwaypoints()
    print("waypoint logger initialized")
    rclpy.spin(logway)
    # when the garbage collector destroys the node object)
    logway.destroy_node()
    file.close()
    rclpy.shutdown()

if __name__ == 'main':
    main()

