import sys
import rclpy
from rclpy.node import Node

# Import our custom service interface
from my_interfaces.srv import AddTwoInts


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # Create a client for the 'add_two_ints' service.
        # The service type must match the one used by the server.
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Check if the service is available before sending a request.
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Create a request object.
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """Sends a request to the service server."""
        self.req.a = a
        self.req.b = b
        # Call the service asynchronously. This returns a 'future' object.
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    # Check for the correct number of command-line arguments.
    if len(sys.argv) != 3:
        print('Usage: ros2 run py_pubsub service_client <int> <int>')
        return

    # Create the client node.
    minimal_client = MinimalClientAsync()

    # Send the request using the numbers from the command line.
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    # Log the result.
    minimal_client.get_logger().info(
        f'Result of add_two_ints: for {
            sys.argv[1]} + {sys.argv[2]} = {response.sum}'
    )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
