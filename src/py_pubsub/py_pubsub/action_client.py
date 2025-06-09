import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from my_interfaces.action import CountDown


class CountDownActionClient(Node):
    """An action client to call the countdown action."""

    def __init__(self):
        """Initialize the action client node."""
        super().__init__('countdown_action_client')
        self._action_client = ActionClient(self, CountDown, 'countdown')

    def send_goal(self, target_number):
        """Send a goal to the action server."""
        goal_msg = CountDown.Goal()
        goal_msg.target_number = target_number

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(
            f'Sending goal request: Countdown from {target_number}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the server's response to the goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the final result from the action server."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.status}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle feedback messages from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_number}')


def main(args=None):
    """Run the main function for the node."""
    rclpy.init(args=args)
    action_client = CountDownActionClient()
    action_client.send_goal(5)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
