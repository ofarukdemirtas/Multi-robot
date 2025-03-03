#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios
import threading

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_teleop')
        self.vel_pub = self.create_publisher(Twist, '/X3/cmd_vel', 10)
        self.motor_pub = self.create_publisher(Float64MultiArray, '/X3/command/motor_speed', 10)
        self.motors_running = False
        self.thrust = 0.0
        self.create_timer(0.1, self.publish_motor_speed)

    def publish_motor_speed(self):
        msg = Float64MultiArray()
        msg.data = [self.thrust] * 4
        self.motor_pub.publish(msg)

    def takeoff(self):
        # Start with lower thrust and increase gradually
        self.thrust = 150.0
        self.motors_running = True
        self.get_logger().info(f'Motors starting at: {self.thrust}')

    def land(self):
        self.thrust = 0.0
        self.motors_running = False
        self.move(0, 0, 0, 0)  # Stop all movement
        self.get_logger().info('Landing - Motors stopped')

    def adjust_thrust(self, increase=True):
        if increase:
            self.thrust = min(500.0, self.thrust + 10.0)
        else:
            self.thrust = max(0.0, self.thrust - 10.0)
        self.get_logger().info(f'Thrust: {self.thrust}')

    def move(self, x=0.0, y=0.0, z=0.0, angular=0.0):
        if not self.motors_running:
            return
            
        msg = Twist()
        msg.linear.x = float(x * 0.5)  # Reduced velocity multiplier
        msg.linear.y = float(y * 0.5)
        msg.linear.z = float(z * 0.5)
        msg.angular.z = float(angular * 0.5)
        self.vel_pub.publish(msg)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    drone = DroneController()
    
    thread = threading.Thread(target=lambda: rclpy.spin(drone))
    thread.start()
    
    try:
        drone.get_logger().info("""
X3 Drone Control:
---------------------------
t - Start motors
l - Stop motors
i/k - Increase/decrease thrust (small steps)
r/f - Up/Down (when stable)
w/s - Forward/Back
a/d - Left/Right
q/e - Rotate

CTRL-C to quit
""")
        while True:
            key = getKey(settings)
            
            if key == '\x03':  # CTRL-C
                break
                
            elif key == 't':
                drone.takeoff()
                
            elif key == 'l':
                drone.land()
                
            elif key == 'i':
                drone.adjust_thrust(increase=True)
                
            elif key == 'k':
                drone.adjust_thrust(increase=False)
                
            elif key == 'r':  # Move up
                drone.move(z=1.0)
                
            elif key == 'f':  # Move down
                drone.move(z=-1.0)
                
            elif key == 'w':
                drone.move(x=1.0)
                
            elif key == 's':
                drone.move(x=-1.0)
                
            elif key == 'a':
                drone.move(y=1.0)
                
            elif key == 'd':
                drone.move(y=-1.0)
                
            elif key == 'q':
                drone.move(angular=1.0)
                
            elif key == 'e':
                drone.move(angular=-1.0)
                
            else:
                # Stop movement if no movement key is pressed
                if key != '':
                    drone.move()
    except Exception as e:
        drone.get_logger().error(f'Error: {e}')
    finally:
        drone.land()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()
        thread.join()

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    main()
