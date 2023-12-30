import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self, name):
        super().__init__(name)

        # Create a service
        self.service = self.create_service(AddTwoInts, "add_two_ints", self.service_callback)
        # Logger message
        self.get_logger().info("Service Server is ready")

    def service_callback(self, request, response):
        response.sum = request.a + request.b 
        self.get_logger().info("Incoming request\na: %d b: %d" % (request.a, request.b))
        self.get_logger().info("Sending response: %d" % response.sum)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer("simple_service_server")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()