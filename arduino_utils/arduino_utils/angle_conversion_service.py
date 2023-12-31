#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler

class AngleConversionService(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # service to convert euler angles to quaternions
        self.euler_to_quaternion = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.euler_to_quanternion_converter)
        # service to convert quaternions to euler angles
        self.quaternion_to_euler = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternion_to_euler_converter)

        # Logger message
        self.get_logger().info(f"Angle Conversion Services Ready...")

    def euler_to_quanternion_converter(self, request, response):
        self.get_logger().info(f"Incoming angles roll = {request.roll}, pitch = {request.pitch}, yaw = {request.yaw}")
        (response.x, response.y, response.z, response.w)= quaternion_from_euler(request.roll, request.pitch, request.yaw)
        self.get_logger().info(f"Response: Quaternions a = {response.x}, b = {response.y}"
                                                       f"c = {response.z}, d = {response.z}")
        return response


    def quaternion_to_euler_converter(self, request, response):
        self.get_logger().info(f"Incoming quaternions a = {request.x}, b = {request.y}, c = {request.z}, d = {request.z}")
        (response.roll, response.pitch, response.yaw) = euler_from_quaternion([request.x, request.y, request.z, request.w])
        self.get_logger().info(f"Response: Angles roll = {response.roll}"
                                               f", pitch = {response.pitch}"
                                               f", yaw = {response.yaw}")
        return response

def main(args=None):
    rclpy.init()
    node = AngleConversionService("angle_sonversion_service_server")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()