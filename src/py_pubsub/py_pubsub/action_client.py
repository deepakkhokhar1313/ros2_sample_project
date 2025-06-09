# Import the rclpy library, which is the ROS 2 client library for Python.
import rclpy
# Import the ActionClient class, which allows this node to call an action.
from rclpy.action import ActionClient
# Import the Node class to create our client node.
from rclpy.node import Node

# Import our custom action interface that we defined in the my_interfaces package.
from my_interfaces.action import CountDown

# Define the action client class, inheriting from the base Node class.


class CountDownActionClient(Node):

    def __init__(self):
        # Call the constructor of the parent Node class and name our node 'countdown_action_client'.
        super().__init__('countdown_action_client')
        # Create an ActionClient.
        # It needs to know:
        # 1. The node it belongs to (self).
        # 2. The type of the action (CountDown).
        # 3. The name of the action to call ('countdown'). This must match the server's action name.
        self._action_client = ActionClient(self, CountDown, 'countdown')

    def send_goal(self, target_number):
        """Sends a goal to the action server."""
        # Create an instance of the Goal part of our CountDown action interface.
        goal_msg = CountDown.Goal()
        # Set the 'target_number' field of the goal message to the value provided.
        goal_msg.target_number = target_number

        # Log an informational message and wait until the action server is available.
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Log that we are sending the goal request.
        self.get_logger().info(
            f'Sending goal request: Countdown from {target_number}')

        # Asynchronously send the goal. This call is non-blocking.
        # It returns a 'future' object that will eventually contain a handle to the goal.
        # We also register 'self.feedback_callback' to handle any feedback messages.
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Attach a callback function to be executed once the future is complete (i.e., when the server responds).
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """This function is executed when the server accepts or rejects the goal."""
        # Get the result from the future, which is a 'goal_handle'.
        goal_handle = future.result()
        # Check if the goal was rejected by the server.
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        # If the goal was accepted, log it.
        self.get_logger().info('Goal accepted :)')

        # Now that the goal is accepted, we request the final result.
        # This is also an asynchronous call that returns a future.
        self._get_result_future = goal_handle.get_result_async()
        # Attach another callback to be executed when the final result is available.
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """This function is executed when the final result is available."""
        # Get the final result from the future object.
        result = future.result().result
        # Log the 'status' field of the result message.
        self.get_logger().info(f'Result: {result.status}')
        # Shut down the rclpy context, which will cause the spin() in main() to exit.
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """This function is executed every time feedback is received from the server."""
        # The feedback_msg object contains the feedback field.
        feedback = feedback_msg.feedback
        # Log the 'current_number' field from the feedback message.
        self.get_logger().info(f'Received feedback: {feedback.current_number}')


def main(args=None):
    # Initialize the ROS 2 Python client library.
    rclpy.init(args=args)
    # Create an instance of our action client node.
    action_client = CountDownActionClient()
    # Call the method to send a goal, hardcoding the target number to 5.
    action_client.send_goal(5)
    # Spin the node, keeping it alive so it can process callbacks (for feedback and results).
    rclpy.spin(action_client)


if __name__ == '__main__':
    # This is the standard entry point for a Python script.
    main()
