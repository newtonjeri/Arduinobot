import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arduinobot_msgs.action import Fibonacci

class SimpleActionClient(Node):
    def __init__(self, name):
        super().__init__(name)

        self.action_client = ActionClient(self, Fibonacci, "fibonacci")

        self.action_client.wait_for_server()

        self.goal = Fibonacci.Goal()
        self.goal.order = 10
        self.future = self.action_client.send_goal_async(self.goal, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("GOAL REJECTED !!!")
            return
        
        self.get_logger().info("GOAL ACCEPTED !!!")
        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result - {result.sequence}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Received Feedback - {feedback_msg.feedback.partial_sequence}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleActionClient("simple_action_client")
    rclpy.spin(node)


if __name__ == "__main__":
    main()

