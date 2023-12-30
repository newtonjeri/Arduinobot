#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/add_two_ints.hpp>

class SimpleServiceServer: public rclcpp::Node{

    private:
        rclcpp::Service<arduinobot_msgs::srv::AddTwoInts>::SharedPtr service_;

        void serviceCallback(
            const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Request> request,
            const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Response> response
        ){
            RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld   " "b: %ld", request->a, request->b);
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", response->sum);
        }

    public:
        SimpleServiceServer(): Node("simple_service_server"){
            service_ = create_service<arduinobot_msgs::srv::AddTwoInts>(
                "add_two_ints", std::bind(&SimpleServiceServer::serviceCallback, this, std::placeholders::_1, std::placeholders::_2)
            );

            RCLCPP_INFO(this->get_logger(), "Service add_two_ints started");

        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}