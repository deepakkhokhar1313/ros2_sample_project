import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Import all our custom interfaces
from my_interfaces.action import MoveRobot
from my_interfaces.srv import GetStatus
from std_msgs.msg import Float64MultiArray # For publishing position

class RobotControllerNode(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # --- Parameters ---
        self.declare_parameter('robot_speed', 1.0)

        # --- State Variables ---
        self._current_position = [0.0, 0.0]
        self._current_status = "IDLE"

        # --- Action Server for MoveRobot ---
        self._move_robot_action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            self.execute_move_robot_callback)

        # --- Service Server for GetStatus ---
        self._get_status_service = self.create_service(
            GetStatus,
            'get_status',
            self.get_status_callback)

        # --- Publisher for Current Position ---
        self._position_publisher = self.create_publisher(
            Float64MultiArray,
            'current_position',
            10)

        self.get_logger().info('Robot Controller has started.')
        self.get_logger().info('Current Status: IDLE')
        self.get_logger().info('Current Position: [0.0, 0.0]')

    def get_status_callback(self, request, response):
        """Callback for the GetStatus service."""
        self.get_logger().info('GetStatus service called')
        response.status = self._current_status
        return response

    def execute_move_robot_callback(self, goal_handle):
        """Callback to execute the MoveRobot action goal."""
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        
        self.get_logger().info(f'Executing goal: Move to ({target_x}, {target_y})')
        self._current_status = "MOVING"

        feedback_msg = MoveRobot.Feedback()
        
        # Simulate movement
        while rclpy.ok() and self._current_position != [target_x, target_y]:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                self._current_status = "IDLE"
                return MoveRobot.Result(status="Canceled")

            # Simple linear movement logic
            step = self.get_parameter('robot_speed').get_parameter_value().double_value * 0.1
            if self._current_position[0] < target_x: self._current_position[0] += step
            if self._current_position[0] > target_x: self._current_position[0] -= step
            if self._current_position[1] < target_y: self._current_position[1] += step
            if self._current_position[1] > target_y: self._current_position[1] -= step
            
            # Clamp to target to avoid overshooting
            if abs(self._current_position[0] - target_x) < step: self._current_position[0] = target_x
            if abs(self._current_position[1] - target_y) < step: self._current_position[1] = target_y

            # Publish current position on the topic
            pos_msg = Float64MultiArray()
            pos_msg.data = self._current_position
            self._position_publisher.publish(pos_msg)

            # Publish feedback for the action
            feedback_msg.current_x = self._current_position[0]
            feedback_msg.current_y = self._current_position[1]
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)

        goal_handle.succeed()
        self._current_status = "IDLE"
        self.get_logger().info('Goal reached!')
        
        result = MoveRobot.Result()
        result.status = "Goal Reached"
        return result


def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = RobotControllerNode()
    rclpy.spin(robot_controller_node)
    robot_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
