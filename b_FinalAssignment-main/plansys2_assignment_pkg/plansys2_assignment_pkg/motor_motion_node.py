#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

MAX_VEL = 0.5
class MotorControl(Node):

    def __init__(self):
        super().__init__('motor_control')
        # SUBSCRIBER TO ODOM
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription
        # SUBSCRIBER TO CAMERA_THETA_GOAL
        self.subscription = self.create_subscription(
            Bool,
            'motor_rotation_on_off',
            self.rotation_on_off_callback,
            10)
        self.subscription
        # PUBLISHER TO CMD_VEL
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # FLAG VARIABLE
        self.theta = 0.0
        self.theta_goal = 0.0
        self.flag = -1
        self.dt = 0.1
        self.marker_reached = 0
        # TIMER
        self.control_loop_timer = self.create_timer(self.dt, self.robot_movement)

    def robot_movement(self):
        # Main control loop
        if self.marker_reached == 4:
            # kill the node
            self.get_logger().info('ROBOT MOTION NODE KILLED')
            self.destroy_node()
            rclpy.shutdown()
        else:
            if self.flag == 0:
                # rotate the robot
                self.rotate(1)
            elif self.flag == 1:
                # increase the marker reached
                self.marker_reached += 1
                self.flag = -1
            elif self.flag == -1:
                # wait for another marker
                self.flag = -1

    def odom_callback(self, msg):
        # callback for updating the robot orientation
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # convert quaternion to theta
        self.theta = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if self.theta < 0:
            self.theta = math.pi + (math.pi + self.theta)

    def rotation_on_off_callback(self, msg):
        # callback for updating the goal orientation
        mode = msg.data
        # the robot start the rotation
        if mode == True:
            self.flag = 0
        # the robot stop the rotation
        elif mode == False and self.flag == 0:
            self.stop()
            time.sleep(self.dt * 2)
            self.flag = 1
            time.sleep(self.dt * 2)           

    def stop(self):
        # stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)
        time.sleep(0.1)

    def rotate(self, sign):
        # rotate the robot clockwise or counterclockwise
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = sign * MAX_VEL
        self.publisher_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    try:
        motor_control = MotorControl()
        print("Motor control node started")
        
        executor = MultiThreadedExecutor()
        
        executor.add_node(motor_control)
        print("Motor control node added to executor")
        
        try:
            while rclpy.ok():
                executor.spin_once()
        finally:
            rclpy.shutdown()
    except Exception as e:
        print(f"Error in the main motor_control_node.py: {str(e)}")

if __name__ == '__main__':
    main()

