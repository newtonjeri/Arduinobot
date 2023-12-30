import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)

        # topic
        self.subscribe_topic = "simple_line"

        # subscriber
        self.subsciber = self.create_subscription(String, self.subscribe_topic, self.msg_callback, 10)

        # Feedback
        self.get_logger().info("Simple Subscriber has started")

        
    def msg_callback(self, msg):
        self.get_logger().info("Message received: " + msg.data)

    
def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber("simple_subscriber")
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
        
