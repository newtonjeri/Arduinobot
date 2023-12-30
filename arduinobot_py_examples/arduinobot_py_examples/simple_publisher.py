import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self, name):
        super().__init__(name)

        # topic
        self.publish_topic = "simple_line"

        # publisher
        self.publisher_ = self.create_publisher(String, self.publish_topic, 10)

        # timer
        self.timer_period = 2.0

        # timer callback
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # counter
        self.counter = 0

        self.get_logger().info("Simple Publisher Started")

    # Callback function
    def timer_callback(self):
        msg = String()
        msg.data = "Simple Publisher publishing message " + str(self.counter)
        self.publisher_.publish(msg)
        self.counter = self.counter + 1

    

def main():
    rclpy.init()                # Initialise ros2 communication
    simple_publisher = SimplePublisher("simple_publisher_node")
    rclpy.spin(simple_publisher) # keep the node up and ruuning
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()