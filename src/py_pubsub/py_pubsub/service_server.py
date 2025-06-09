# Import the necessary ROS 2 libraries
import rclpy
from rclpy.node import Node

# Import our custom service interface
from my_interfaces.srv import AddTwoInts


class MinimalService(Node):

    def __init__(self):
        # Initialize the Node with the name 'minimal_service'
        super().__init__('minimal_service')

        # Create the service. This is the core of the service server.
        self.srv = self.create_service(
            AddTwoInts,                 # The service type
            'add_two_ints',             # The name of the service
            self.add_two_ints_callback  # The function to call when a request is received
        )

    def add_two_ints_callback(self, request, response):
        """
        This callback function is executed whenever the service is called.
        It receives the request, performs the logic, and populates the response.
        """
        # The request and response objects are of the type we defined in AddTwoInts.srv

        # Perform the calculation
        response.sum = request.a + request.b

        # Log the request and the result
        self.get_logger().info(
            f'Incoming request\na: {request.a} b: {request.b}\n'
            f'Sending back response: [sum: {response.sum}]'
        )

        # Return the response object
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    # Spin the node to keep it alive and waiting for service requests
    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
