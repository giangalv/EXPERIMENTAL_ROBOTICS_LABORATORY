#!/usr/bin/env python3
import rclpy
from rclpy.parameter import Parameter, ParameterType
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from std_srvs.srv import SetBool

class MarkerSearcherAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('find_marker', 0.5)
        self.action_name = 'marker_searcher'
        self.service = self.create_service(SetBool, 'response_marker_searcher', self.service_callback)
        self.client = self.create_client(SetBool, 'search_marker')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.state = 0

    def callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed: %s' % str(e))
    
    def service_callback(self, request, response):
        self.state = 2
        response.success = True
        return response

    def do_work(self):
        if self.state == 0:
            self.state = 1
            request = SetBool.Request()
            request.data = True
            future = self.client.call_async(request)
            future.add_done_callback(self.callback)
        elif self.state == 1:
            self.state = 1
        else:
            self.state = 0
            self.finish(True, 1.0, 'Charge completed')

def main(args=None):
    rclpy.init(args=args)
    node = MarkerSearcherAction()
    node.set_parameters([Parameter(name='action_name', value='find_marker')])
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()