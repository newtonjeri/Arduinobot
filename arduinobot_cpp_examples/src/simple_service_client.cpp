#include <rclcpp/rclcpp.hpp>
#include <memory>
#include<chrono>
#include <arduinobot_msgs/srv/add_two_ints.hpp>

using namespace std::chrono_literals;

class SimpleServiceCLient: public rclcpp::Node{
    private:
        rclcpp::Client<arduinobot_msgs::srv::AddTwoInts>::SharedPtr client_;

        void responseCallback(rclcpp::Client<arduinobot_msgs::srv::AddTwoInts>::SharedFuture future){
            if(future.valid()){
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Incoming Response = " << future.get()->sum);
            }else{
                RCLCPP_ERROR(this->get_logger(), "Failed to get response");
            }
        }
    public:
        SimpleServiceCLient(int a, int b): Node("simple_service_client"){

            client_ = create_client<arduinobot_msgs::srv::AddTwoInts>("add_two_ints");
            auto request = std::make_shared<arduinobot_msgs::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            // Wait for server
            while(!client_->wait_for_service(1s)){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting");
                    return;
                }
                RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
            }

            auto result = client_->async_send_request(request, std::bind(&SimpleServiceCLient::responseCallback, this, std::placeholders::_1));

        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    if (argc !=3){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node started with wrong number of arguments");
        return 1;
    }
    auto node = std::make_shared<SimpleServiceCLient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}