#!/usr/bin/python3

import rclpy
from rclpy.parameter import Parameter
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from std_srvs.srv import SetBool

class GoToAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('go_to_spot', 0.5)      
        self.service = self.create_service(SetBool, 'response_go_to', self.service_callback)
        self.client = self.create_client(SetBool, 'go_to_spot')
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
    node = GoToAction()
    node.set_parameters([Parameter(name='action_name', value='go_to_spot')])
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

