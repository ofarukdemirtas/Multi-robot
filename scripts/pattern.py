#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import math
import numpy as np
from transforms3d.euler import quat2euler

class CircleAndFollowControl(Node):
    def __init__(self):
        super().__init__('circle_and_follow_control')
        
        # Publishers
        self.ugv_pub = self.create_publisher(Twist, '/model/x1/cmd_vel', 10)
        self.drone_pub = self.create_publisher(Twist, '/X3/cmd_vel', 10)
        self.motor_pub = self.create_publisher(Float64MultiArray, '/X3/command/motor_speed', 10)
        
        # Subscribers
        self.ugv_odom_sub = self.create_subscription(Odometry, '/model/x1/odometry', self.ugv_odom_callback, 10)
        self.uav_odom_sub = self.create_subscription(Odometry, '/model/X3/odometry', self.uav_odom_callback, 10)
        
        # UGV parameters (small circle)
        self.linear_speed = 0.3
        self.angular_speed = 0.5  
        
        # UAV parameters
        self.desired_altitude = 3.0
        self.hover_thrust = 700.0  
        self.base_motor_speed = [self.hover_thrust] * 4
        
        # Altitude PID
        self.kp_z = 15.0  # Thrust per meter error
        self.ki_z = 5.0
        self.kd_z = 15.0
        self.integral_z = 0.0
        self.prev_error_z = 0.0
        
        # Position storage
        self.ugv_position = {'x': 0.0, 'y': 0.0, 'z': 0.1}
        self.uav_position = {'x': 0.0, 'y': 0.0, 'z': 3.0}
        
        # Timers
        self.ugv_timer = self.create_timer(0.05, self.move_ugv)
        self.uav_timer = self.create_timer(0.02, self.control_uav)
        
        self.get_logger().info('Altitude-stabilized follow node initialized')

    def ugv_odom_callback(self, msg):
        self.ugv_position['x'] = msg.pose.pose.position.x
        self.ugv_position['y'] = msg.pose.pose.position.y
        self.get_logger().debug(f'UGV position: x={self.ugv_position["x"]:.2f}, y={self.ugv_position["y"]:.2f}')

    def uav_odom_callback(self, msg):
        self.uav_position['x'] = msg.pose.pose.position.x
        self.uav_position['y'] = msg.pose.pose.position.y
        self.uav_position['z'] = msg.pose.pose.position.z
        self.get_logger().debug(f'UAV position: x={self.uav_position["x"]:.2f}, y={self.uav_position["y"]:.2f}, z={self.uav_position["z"]:.2f}')

    def move_ugv(self):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_speed
        self.ugv_pub.publish(cmd)
        self.get_logger().debug('UGV command sent: linear.x={:.2f}, angular.z={:.2f}'.format(cmd.linear.x, cmd.angular.z))

    def control_uav(self):
        # Altitude PID control
        error_z = self.desired_altitude - self.uav_position['z']
        dt = 0.02  # Timer period
        
        # PID terms
        self.integral_z += error_z * dt
        derivative_z = (error_z - self.prev_error_z) / dt
        
        # Anti-windup
        self.integral_z = np.clip(self.integral_z, -100.0, 100.0)
        
        # Total thrust adjustment
        thrust_adj = (self.kp_z * error_z) + (self.ki_z * self.integral_z) + (self.kd_z * derivative_z)
        motor_speeds = [self.hover_thrust + thrust_adj] * 4
        
        # Clamp motor speeds to [400, 900] to prevent runaway
        motor_speeds = np.clip(motor_speeds, 400.0, 900.0).tolist()
        
        # Publish motor speeds
        motor_msg = Float64MultiArray()
        motor_msg.data = motor_speeds
        self.motor_pub.publish(motor_msg)
        
        # XY position control (velocity-based)
        error_x = self.ugv_position['x'] - self.uav_position['x']
        error_y = self.ugv_position['y'] - self.uav_position['y']
        
        xy_cmd = Twist()
        xy_cmd.linear.x = np.clip(1.5 * error_x, -1.0, 1.0)
        xy_cmd.linear.y = np.clip(1.5 * error_y, -1.0, 1.0)
        self.drone_pub.publish(xy_cmd)
        
        self.prev_error_z = error_z
        self.get_logger().debug(f'UAV control: error_z={error_z:.2f}, motor_speeds={motor_speeds}, xy_cmd=({xy_cmd.linear.x:.2f}, {xy_cmd.linear.y:.2f})')

def main():
    rclpy.init()
    node = CircleAndFollowControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robots when interrupted
        stop_cmd = Twist()
        node.ugv_pub.publish(stop_cmd)
        node.drone_pub.publish(stop_cmd)
        node.get_logger().info('Stopping robots and shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()