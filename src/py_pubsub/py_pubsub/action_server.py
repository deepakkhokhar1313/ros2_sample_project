import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Import our custom action interface
from my_interfaces.action import CountDown

class CountDownActionServer(Node):

    def __init__(self):
        super().__init__('countdown_action_server')
        self._action_server = ActionServer(
            self,
            CountDown, # The action type.
            'countdown',# The name of the action that clients will use to connect.
            self.execute_callback)

    def execute_callback(self, goal_handle):
        """This callback is executed when a new goal is accepted."""
        self.get_logger().info('Executing goal...')

        # Create a feedback message
        feedback_msg = CountDown.Feedback()

        # Start the countdown
        for i in range(goal_handle.request.target_number, 0, -1):
            # Check if the client has requested a cancelation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return CountDown.Result()

            # Update and publish feedback
            feedback_msg.current_number = i
            self.get_logger().info(f'Feedback: {feedback_msg.current_number}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Wait for one second
            time.sleep(1)

        # When the loop is finished, set the goal as succeeded
        goal_handle.succeed()

        # Populate and return the result
        result = CountDown.Result()
        result.status = 'Countdown finished successfully!'
        return result


def main(args=None):
    rclpy.init(args=args)
    countdown_action_server = CountDownActionServer()
    # Use a MultiThreadedExecutor to allow multiple callbacks to run concurrently
    # This is important for actions so they don't block other ROS 2 activity
    rclpy.spin(countdown_action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
