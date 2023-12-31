#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/quaternion_to_euler.hpp>
#include <arduinobot_msgs/srv/euler_to_quaternion.hpp>
#include <tf2/utils.h>


class AngleConversionService : public rclcpp::Node
{
    private:
        rclcpp::Service<arduinobot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_service;
        rclcpp::Service<arduinobot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion_service;

    public:
        AngleConversionService(): Node("angle_conversion_service"){
            // Quaternion to euler service
            quaternion_to_euler_service = create_service<arduinobot_msgs::srv::QuaternionToEuler>( "quaternion_to_euler", std::bind(&AngleConversionService::quaternionToEulerCallback, this, std::placeholders::_1, std::placeholders::_2));
            // Euler to quaternion service
            euler_to_quaternion_service = create_service<arduinobot_msgs::srv::EulerToQuaternion>("euler_to_quaternion", std::bind(&AngleConversionService::eulerToQuaternionCallback, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Angle Conversion Services are ready ...");
        }


        void quaternionToEulerCallback(
            const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Request> request,
            const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Response> response
        ){
            RCLCPP_INFO_STREAM(this->get_logger(), "Received quaternion a = " << request->x << "b = " << request->y << "c = " << request->z <<"d = " << request->w);
            tf2::Quaternion q(request->x, request->y, request->z, request->w);
            tf2::Matrix3x3 m(q);
            m.getRPY(response->roll, response->pitch, response->yaw);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding Euler Angles: roll=" << response->roll << " pitch=" << response->pitch << " yaw=" << response->yaw);

        }

        void eulerToQuaternionCallback(
            const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Request> request,
            const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Response> response
        ){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received Euler Angles: roll=" << request->roll << " pitch=" << request->pitch << " yaw=" << request->yaw);
            tf2::Quaternion q;
            q.setRPY(request->roll, request->pitch, request->yaw);
            response->x = q.getX();
            response->y = q.getY();
            response->z = q.getZ();
            response->w = q.getW();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding quaternion: a=" << response->x << " b=" << response->y << " c=" << response->z << " d=" << response->w);

        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AngleConversionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
