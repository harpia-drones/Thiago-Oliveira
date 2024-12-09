import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from example_interfaces.srv import SetBool
from statistics import mean

class TemperatureMonitorNode(Node):
    def __init__(self):
        super().__init__('temperature_monitor')

        # create a subscriber to receive the temperatures
        self.subscriber_ = self.create_subscription(
            Float64,                                    # msg type
            'temperature',                              # topic name
            self.callback_temperature_subscriber,       # callback to receive the msg
            10                                          # queue size  
        )

        # create a publish to publish the average of last five temperatures
        self.publisher_ = self.create_publisher(
            Float64,                                   # msg type
            'average_temperature',                     # topic name
            10                                         # queue size
        )

        # create a service
        self.srv = self.create_service(
            SetBool, 
            'reset_average', 
            self.reset_callback
        )

        # call the callback function to publish the average to the topic 
        self.timer_ = self.create_timer(0.5, self.callback_temperature_avg_monitor)

        # store the last five temperatures
        self.last_five_temperatures_ = [0, 0, 0, 0, 0]

    # callback for subscriber: 
    def callback_temperature_subscriber(self, msg):

        '''
        inserts the most recent temperature at position 0 and shifts the other value to the right, adding one more position
        at the end of the list, and then remove the remaining position
        '''

        self.last_five_temperatures_.insert(0, msg.data) 
        self.last_five_temperatures_.pop()

        # # debbug
        # self.get_logger().info(f'Temperatures received = {self.last_five_temperatures_}')
    
    # publish the average to the /average_temperature topic
    def callback_temperature_avg_monitor(self):
        msg = Float64()

        if self.last_five_temperatures_[4] != 0: 
            msg.data = mean(self.last_five_temperatures_)
            self.publisher_.publish(msg)

    # reset the average
    def reset_callback(self, request, response):
        if request.data:  # Reinicia o cálculo da média se solicitado
            self.last_five_temperatures_ = [0, 0, 0, 0, 0]
            self.get_logger().info('Average restarted')
            response.success = True
            response.message = 'Average restarted succesfully.'
        else:
            response.success = False
            response.message = 'Nothing done.'
        return response
        
def main(args=None):
    rclpy.init(args=args)

    node = TemperatureMonitorNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()