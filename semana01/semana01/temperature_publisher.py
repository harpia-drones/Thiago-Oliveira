import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import sin
from random import randint

class TemperaturePublisherNode(Node):

    def __init__(self):
        super().__init__("temperature_publisher")

        # create the publisher
        self.publisher_ = self.create_publisher(
            Float64,             # msg type 
            "temperature",       # topic name
            10                   # queue size
        )

        # call the callback function at a rate of 2 Hz
        self.timer_ = self.create_timer(0.5, self.callback_temperature_publisher)

    # publish data over the topic 
    def callback_temperature_publisher(self):
        temperature = self.generate_temperature()
        
        msg = Float64()                  # create the msg
        msg.data = temperature           # assigns the temperature to the msg
        self.publisher_.publish(msg)     # publish the msg
        
        # debbug
        # self.get_logger().info(f'temperature published = {msg.data}')
    
    # create the temperature generation function
    def generate_temperature(self):
        random_number = randint(80, 100)
        temperature = 25 + 5 * sin(random_number/10)
        return temperature 
    
def main(args=None):
    rclpy.init(args=args)

    node = TemperaturePublisherNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()