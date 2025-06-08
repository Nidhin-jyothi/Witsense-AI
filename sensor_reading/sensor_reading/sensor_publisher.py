import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publsiher_ = self.create_publisher(Float64, 'sensor_x_reading', 10)
        self.timer = self.create_timer(1.0, self.publish_random_reading)

    def publish_random_reading(self):
        reading = random.random()
        msg = Float64()
        msg.data = reading
        self.publsiher_.publish(msg)
        self.get_logger().info(f'Published: {reading:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()












                