import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    """A minimal publisher node that uses parameters."""

    def __init__(self):
        """Initialize the publisher node."""
        super().__init__('minimal_publisher')
        self.declare_parameter('publish_message', 'Default Hello World')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        """Periodically get parameter and publish a message."""
        my_param = self.get_parameter(
            'publish_message').get_parameter_value().string_value
        msg = String()
        msg.data = f'{my_param}: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    """Run the main function for the node."""
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
