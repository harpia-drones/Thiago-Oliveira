import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperatureAverageMonitorNode(Node):
    def __init__(self):
        super().__init__("temperature_avg_monitor")

        # create a subscriber
        self.subscriber_ = self.create_subscription(
            Float64,
            "average_temperature",
            self.callback_subscriber,
            10
        )

    # print the average temperature
    def callback_subscriber(self, msg):
        self.get_logger().info(f'{msg.data}')

def main(args=None):
    rclpy.init(args=args)

    node = TemperatureAverageMonitorNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()