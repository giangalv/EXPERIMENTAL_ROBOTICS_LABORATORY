#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from rclpy.node import Node
from std_msgs.msg import Bool

from std_srvs.srv import SetBool
from ros2_aruco_interfaces.msg import ArucoMarkers
import math

class CameraCheck(Node):

    def __init__(self):
        super().__init__('camera_check')    
        self.srv_client_marker_searcher = self.create_client(SetBool, 'response_marker_searcher')
        self.service = self.create_service(SetBool, 'search_marker', self.service_callback) 
        while not self.srv_client_marker_searcher.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            
        self.active = False
         # SUBSCRIBER TO ARUCO_MARKERS
        self.subscription_aruco = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10)
        self.subscription_aruco  # prevent unused variable warning
        # PUBBLISHER TO MOTOR for rotating the robot
        self.publisher_robot_onoff = self.create_publisher(Bool, 'motor_rotation_on_off', 10)
        # TIMER for doing the controller logic
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)       
        ## INTERNAL VARIABLES ##
        self.id_marker = 0
        # list of the markers to reach
        self.goal_markers = [11, 12, 13, 15]
        # number of the marker reached
        self.reached_marker = 0
        # flag for the logic
        self.flag = 0
        # flag for the marker saw
        self.flag_marker = 0
        
##############################################################################
############################# DEBUG FUNCTION #################################
##############################################################################
    def wait_for_input(self):
            while True:
                user_input = input("Quando vuoi andare avanti: ")
                if user_input:
                    break
                
    ###### DEBUG line ######
    #self.wait_for_input()
    ########################
                
##############################################################################
############################# FUNCTIONS ######################################
##############################################################################
        
    ## callback for STARTING the functions ##
    def service_callback(self, request, response):
        self.active = request.data
        response.success = True
        return response
             
    ## TIMER for the CONTROLLER LOGIC ##
    def timer_callback(self):
        if self.active:
            # check if all the markers are reached         
            self.id_marker = self.goal_markers[self.reached_marker] # take the marker's id to reach
            if self.flag == 0:
                self.rotation_robot_activation(True)
                if self.flag_marker == 1:
                    self.rotation_robot_activation(False)
                    self.flag = 1
            elif self.flag == 1:
                self.get_logger().info('MARKER {} FOUND'.format(self.id_marker))
                self.reached_marker += 1
                if self.reached_marker == len(self.goal_markers):
                    self.get_logger().info('ALL MARKERS REACHED')
                    self.end_iteration()
                    # kill the node
                    self.destroy_node()
                    rclpy.shutdown()  
                else:
                    self.flag_marker = 0
                    self.flag = 0
                    self.end_iteration()          
    
    ## callback for ENDING the iteration ##           
    def end_iteration(self):
        if self.active:
            self.active = False
            request = SetBool.Request()
            request.data = True
            future = self.srv_client_marker_searcher.call_async(request)
            future.add_done_callback(self.callback)
    
    ## callback for the SERVICE ##    
    def callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed: %s' % str(e))                    
    
    ## PUBLISHER to CAMERA for rotating itself in searching mode ##
    def rotation_robot_activation(self, on_off):
        msg = Bool()
        msg.data = on_off
        self.publisher_robot_onoff.publish(msg)  
                   
    ## callback for UPDATE the MARKER'S INFO ##
    def aruco_callback(self, msg):
        if self.reached_marker < len(self.goal_markers):
            # take the marker's id to reach
            self.id_marker = self.goal_markers[self.reached_marker]
        # take the markers's id
        self.ids_marker = msg.marker_ids
        # check if the marker is the one we are looking for
        if self.id_marker in self.ids_marker:
            self.flag_marker = 1        
        else:
            # if the marker is not in the list we wait and then we check again
            self.flag_marker = 0
                

def main(args=None):
    rclpy.init(args=args)
    try:
        camera_check = CameraCheck()
        print("Robot Control node has been started")
        
        # Create the executor
        executor = MultiThreadedExecutor()
        
        # Add the node to the executor
        executor.add_node(camera_check)
        print("Robot Control node has been added to the executor")
        
        try:
            while rclpy.ok():
                executor.spin_once()
        finally:
            rclpy.shutdown()
    except Exception as e:
        print(f"ERROR in the main camera_check_node.py: {str(e)}")

if __name__ == '__main__':
    main()