import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class AverageResetClientNode(Node):
    def __init__(self):
        super().__init__('average_reset_client')

        self.send_request()

    def send_request(self):
        '''
        Send a request to the service asking for reset the average count. 
        '''
        
        self.client = self.create_client(
            SetBool,             # srv interface type
            'reset_average'     # service
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for the reset_average... service.')

        request = SetBool.Request()
        request.data = True      # Solicitação para reiniciar a média
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().info('Failed to reset the average.')
        except Exception as e:
            self.get_logger().error(f'Error when calling the service: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AverageResetClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()