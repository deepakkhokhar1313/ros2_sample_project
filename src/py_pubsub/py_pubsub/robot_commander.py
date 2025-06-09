import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import all our custom interfaces and standard messages
from my_interfaces.action import MoveRobot
from my_interfaces.srv import GetStatus
from std_msgs.msg import Float64MultiArray

class RobotCommanderNode(Node):

    def __init__(self):
        super().__init__('robot_commander')

        # --- Action Client for MoveRobot ---
        self._move_robot_client = ActionClient(self, MoveRobot, 'move_robot')

        # --- Service Client for GetStatus ---
        self._get_status_client = self.create_client(GetStatus, 'get_status')
        # Wait for the service to be available
        while not self._get_status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetStatus service not available, waiting again...')

        # --- Subscriber for Current Position ---
        self._position_subscriber = self.create_subscription(
            Float64MultiArray,
            'current_position',
            self.position_callback,
            10)

        # --- Timer for periodic status checks ---
        self._status_timer = self.create_timer(2.0, self.check_robot_status)

        self.get_logger().info('Robot Commander has started.')

    def position_callback(self, msg):
        """Callback for the /current_position topic."""
        # This logs the position but doesn't do much else for this example.
        # In a real system, this could update a UI or trigger other logic.
        position = [round(p, 2) for p in msg.data]
        self.get_logger().info(f'Received position: {position}', throttle_duration_sec=1)

    def check_robot_status(self):
        """Callback for the timer to periodically check robot status."""
        request = GetStatus.Request()
        future = self._get_status_client.call_async(request)
        future.add_done_callback(self.status_response_callback)

    def status_response_callback(self, future):
        """Callback for when the GetStatus service responds."""
        try:
            response = future.result()
            self.get_logger().info(f'Periodic Status Check: Robot is {response.status}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e!r}')

    def send_move_goal(self, x, y):
        """Sends a goal to the MoveRobot action server."""
        self.get_logger().info('Waiting for move_robot action server...')
        self._move_robot_client.wait_for_server()

        goal_msg = MoveRobot.Goal()
        goal_msg.target_x = float(x)
        goal_msg.target_y = float(y)

        self.get_logger().info(f'Sending move goal request to ({x}, {y})')
        self._send_goal_future = self._move_robot_client.send_goal_async(
            goal_msg,
            feedback_callback=self.move_feedback_callback)

        self._send_goal_future.add_done_callback(self.move_goal_response_callback)

    def move_goal_response_callback(self, future):
        """Callback for when the server accepts or rejects the move goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Move goal rejected :(')
            return

        self.get_logger().info('Move goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.move_get_result_callback)

    def move_get_result_callback(self, future):
        """Callback for when the final result of the move action is available."""
        result = future.result().result
        self.get_logger().info(f'Move Result: {result.status}')
        # We can add logic here to send another goal, or shut down.
        # For now, we'll just stop the status timer.
        self._status_timer.cancel()
        rclpy.shutdown()

    def move_feedback_callback(self, feedback_msg):
        """Callback for receiving feedback from the move action."""
        position = [round(p, 2) for p in [feedback_msg.feedback.current_x, feedback_msg.feedback.current_y]]
        self.get_logger().info(f'Received move feedback: At position {position}')


def main(args=None):
    rclpy.init(args=args)
    robot_commander = RobotCommanderNode()
    # Send an initial goal to move the robot to position (5.0, 5.0)
    robot_commander.send_move_goal(15.0, 5.0)
    rclpy.spin(robot_commander)


if __name__ == '__main__':
    main()
