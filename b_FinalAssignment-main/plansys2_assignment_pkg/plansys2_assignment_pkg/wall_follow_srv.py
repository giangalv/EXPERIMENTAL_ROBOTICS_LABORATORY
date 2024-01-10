#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

active_ = False
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

class ReadingLaser(Node):
    def __init__(self):
        super().__init__('reading_laser')
        
        global pub_
        pub_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub = self.create_subscription(LaserScan, '/laser/scan', self.clbk_laser, 1)
        self.srv = self.create_service(SetBool, 'wall_follower_switch', self.wall_follower_switch)
        
        self.active_ = False
        
        self.get_logger().info('ReadingLaser node initialized')

        self.timer = self.create_timer(0.1, self.run)

    def wall_follower_switch(self, req, res):
        global active_
        self.active_ = req.data
        res.success = True
        return res

    def clbk_laser(self, msg):
        global regions_
        regions_ = {
            'right': min(min(msg.ranges[180:251]), 10),
            'fright': min(min(msg.ranges[252:323]), 10),
            'front': min(min(msg.ranges[324:395]), 10),
            'fleft': min(min(msg.ranges[396:467]), 10),
            'left': min(min(msg.ranges[468:539]), 10),
        }
        self.take_action()

    def change_state(self, state):
        global state_, state_dict_
        if state is not state_ and self.active_:
            self.get_logger().info(f'Wall follower - [{state}] - {state_dict_[state]}')
            state_ = state

    def take_action(self):
        global regions_
        regions = regions_

        d0 = 0.6
        d = 1.0
        
        if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)     
        elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        else:
            state_description = 'unknown case'
            self.get_logger().info(regions)

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3
        return msg

    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.3
        return msg

    def follow_the_wall(self):
        global regions_
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        return msg

    def run(self):
        if self.active_:
            msg = Twist()
            if state_ == 0:
                msg = self.find_wall()
            elif state_ == 1:
                msg = self.turn_left()
            elif state_ == 2:
                msg = self.follow_the_wall()
            else:
                self.get_logger().error('Unknown state!')
            pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    reading_laser_node = ReadingLaser()
    executor = MultiThreadedExecutor()
    executor.add_node(reading_laser_node)
    try:
        executor.spin_once() 
        while rclpy.ok():
            executor.spin_once()
    except KeyboardInterrupt:
        pass
    reading_laser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

