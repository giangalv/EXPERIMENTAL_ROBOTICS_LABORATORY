#!/usr/bin/env python3

import rclpy
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class GoToPoint(Node):
    def __init__(self):
        super().__init__('go_to_point')

        # robot state variables
        self.position = Point()
        self.yaw = 0

        # machine state
        self.state = 0

        # goal
        self.desired_position = Point()
        self.desired_position.x = 0.0
        self.desired_position.y = 0.0
        self.desired_position.z = 0.0
        self.marker_pos = [(6.0, 2.0), (7.0, -5.0), (-3.5, -8.0), (-7.0, 1.5), (1.0, 1.0)]
        self.counter = 0

        # parameters
        self.yaw_precision = math.pi / 9  # +/- 20 degree allowed
        self.yaw_precision_2 = math.pi / 90  # +/- 2 degree allowed
        self.dist_precision = 0.3

        self.kp_a = 3.0
        self.kp_d = 0.2
        self.ub_a = 0.6
        self.lb_a = -0.5
        self.ub_d = 0.6

        # publishers
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # service callbacks
        self.service = self.create_service(SetBool, 'response_go_to', self.service_callback)
        self.srv = self.create_service(SetBool, 'go_to_point_switch', self.go_to_point_switch)

        # subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.clbk_odom, 1)

        self.active = False

        self.get_logger().info('GoToPoint node initialized')

        self.timer = self.create_timer(0.1, self.run)

    def service_callback(self, request, response):
        self.counter += 1
        response.success = True
        return response

    def go_to_point_switch(self, req, res):
        self.active = req.data
        res.success = True
        return res
    
    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if self.yaw < 0:
            self.yaw = math.pi + (math.pi + self.yaw)
        self.yaw = self.normalize_angle(self.yaw)

    def change_state(self, state):
        self.state = state
        self.get_logger().info(f'State changed to [{self.state}]')

    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle
    
    def normalize_angle(self, angle):
        return euler_from_quaternion([0, 0, math.sin(angle / 2), math.cos(angle / 2)])[2]

    def fix_yaw(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)
        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_precision_2:
            twist_msg.angular.z = self.kp_a * err_yaw
            if twist_msg.angular.z > self.ub_a:
                twist_msg.angular.z = self.ub_a
            elif twist_msg.angular.z < self.lb_a:
                twist_msg.angular.z = self.lb_a
        self.pub.publish(twist_msg)
        if math.fabs(err_yaw) <= self.yaw_precision_2:
            self.get_logger().info(f'Yaw error: [{err_yaw}]')
            self.change_state(1)

    def go_straight_ahead(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = desired_yaw - self.yaw
        err_pos = math.sqrt(pow(des_pos.y - self.position.y, 2) + pow(des_pos.x - self.position.x, 2))
        if err_pos > self.dist_precision:
            twist_msg = Twist()
            twist_msg.linear.x = self.kp_d * err_pos
            if twist_msg.linear.x > self.ub_d:
                twist_msg.linear.x = self.ub_d
            twist_msg.angular.z = self.kp_a * err_yaw
            self.pub.publish(twist_msg)
        else:
            self.get_logger().info(f'Position error: [{err_pos}]')
            self.change_state(2)
        if math.fabs(err_yaw) > self.yaw_precision:
            self.get_logger().info(f'Yaw error: [{err_yaw}]')
            self.change_state(0)

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub.publish(twist_msg)

    def run(self):
        if self.active:
            position = self.marker_pos[self.counter]
            x_des, y_des = position
            self.desired_position.x = float(x_des)
            self.desired_position.y = float(y_des)
            if self.state == 0:
                self.fix_yaw(self.desired_position)
            elif self.state == 1:
                self.go_straight_ahead(self.desired_position)
            elif self.state == 2:
                self.done()
            else:
                self.get_logger().error('Unknown state!')

def main(args=None):
    rclpy.init(args=args)
    go_to_point_node = GoToPoint()
    executor = MultiThreadedExecutor()
    executor.add_node(go_to_point_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    go_to_point_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()