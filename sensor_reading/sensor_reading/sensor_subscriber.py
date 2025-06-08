import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            'sensor_x_reading',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        value = msg.data
        if value > 0.5:
            self.get_logger().info(f'Received: {value:.3f} > 0.5')
        else:
            self.get_logger().info(f'Received {value:.3f} <= 0.5')


def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()                    