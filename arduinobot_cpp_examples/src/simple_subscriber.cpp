#include <rclcpp/rclcpp.hpp>
// The message type is defined in the package std_msgs, in the subfolder msg, in the file String.msg
#include <std_msgs/msg/string.hpp>

class SimpleSubscriber: public rclcpp::Node{
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    public:
        SimpleSubscriber(): Node("simple_sunscriber"){
            subscription_ = create_subscription<std_msgs::msg::String>("simple_line_cpp", 10, std::bind(&SimpleSubscriber::msgCallback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Simple Subscriber Started");
        }
        void msgCallback(const std_msgs::msg::String::SharedPtr);
};

void SimpleSubscriber::msgCallback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}