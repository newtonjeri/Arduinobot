import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import AddTwoInts

class SimpleServiceClient(Node):
    def __init__(self, name):
        super().__init__(name)

        # Create a client
        self.client = self.create_client(AddTwoInts, "add_two_ints")

        # Wait for the service to be available
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service add_two_ints to be available")

        self.client_setup()

    
    def client_setup(self):
        # Create a request
        self.request = AddTwoInts.Request()

        # Get the numbers from the user
        self.request.a = int(input("Enter a: "))
        self.request.b = int(input("Enter b: "))
        self.get_logger().info("Sending request\na: %d b: %d" % (self.request.a, self.request.b))
        # Call the service and get the response
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.future_callback)
    
    def future_callback(self, future):
        self.response = future.result().sum
        self.get_logger().info(f"Incoming Response = {self.response}")

def main():
    rclpy.init()
    node = SimpleServiceClient("simple_service_client")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()