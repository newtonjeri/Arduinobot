#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <vector>
#include <memory>


class SimpleParameters: public rclcpp::Node{
    public:
        SimpleParameters(): Node("simple_parameter"){
            declare_parameter<int>("simple_int_parameter", 13);
            declare_parameter<std::string>("simple_str_parameter", "Am Newton");

            param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameters::paramChangeCallback, this, std::placeholders::_1));
        }
    private:
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters){
            auto result = rcl_interfaces::msg::SetParametersResult();
            for(auto parameter: parameters){
                if(parameter.get_name() == "simple_int_parameter"){
                    RCLCPP_INFO(get_logger(), "simple_int_parameter changed from %d to %d", get_parameter("simple_int_parameter").as_int(), parameter.as_int());
                    result.successful = true;
                }
                else if(parameter.get_name() == "simple_str_parameter"){
                    RCLCPP_INFO(get_logger(), "simple_str_parameter changed from %s to %s", get_parameter("simple_str_parameter").as_string().c_str(), parameter.as_string().c_str());
                    result.successful = true;
                }
                else{
                    result.successful = false;
                    result.reason = "Unknown parameter";
                    RCLCPP_INFO(get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
                }
            }
            return result;
        }


};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameters>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

