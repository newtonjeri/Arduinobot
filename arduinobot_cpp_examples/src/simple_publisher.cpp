#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher: public rclcpp::Node{
    public:
        SimplePublisher(): Node("simple_publisher"), counter_(0){
            publisher_ = create_publisher<std_msgs::msg::String>("simple_line_cpp", 10);
            timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));

            RCLCPP_INFO(get_logger(), "Simple Publisher Started");
        }

        void timerCallback();

    private:
        unsigned int counter_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

void SimplePublisher::timerCallback(){
    auto message = std_msgs::msg::String();
    message.data = "Simple Publisher publishing message " + std::to_string(counter_);
    publisher_->publish(message);
    counter_ += 1;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}