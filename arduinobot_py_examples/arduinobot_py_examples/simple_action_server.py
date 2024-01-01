import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_msgs.action import Fibonacci

class SimpleActionServer(Node):

    def __init__(self, name):
        super().__init__(name)
        self.action_server = ActionServer(self, Fibonacci, "fibonacci", self.goal_callback)
        self.get_logger().info("Action serve fibonnaci is ready ...")

    def goal_callback(self, goal_handle):
        self.get_logger().info(f"Received goal request of order {goal_handle.request.order}")
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info(f"Feedback - {format(feedback_msg.partial_sequence)}")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = SimpleActionServer("simple_action_server")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

