#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <thread>

#include <arduinobot_msgs/action/fibonacci.hpp>

using namespace std::placeholders;


namespace arduinobot_action_server{
        
    class SimpleActionServer: public rclcpp::Node{
        private:
            rclcpp_action::Server<arduinobot_msgs::action::Fibonacci>::SharedPtr action_server;

            rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                                    std::shared_ptr<const arduinobot_msgs::action::Fibonacci::Goal> goal){   

                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with order " << goal->order);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                }
    

            void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>> goal_handle){
                std::thread(std::bind(&SimpleActionServer::execute, this, _1), goal_handle).detach();   
            }

            void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>> goal_handle){
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Executing Goal");
                rclcpp::Rate loop_rate(1);

                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<arduinobot_msgs::action::Fibonacci::Feedback>();
                auto &sequence = feedback->partial_sequence;

                sequence.push_back(0);
                sequence.push_back(1);

                auto result = std::make_shared<arduinobot_msgs::action::Fibonacci::Result>();

                for(int i = 1; (i < goal->order) && rclcpp::ok(); i++){

                    if(goal_handle->is_canceling()){
                        result->sequence = sequence;
                        goal_handle->canceled(result);
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Goal Canceled");
                        return;
                    }

                    sequence.push_back(sequence[i] + sequence[i-1]);
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Feedback - " << );
                    loop_rate.sleep();
                }

                if (rclcpp::ok()){
                    result->sequence = sequence;
                    goal_handle->succeed(result);
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Goal Succeeded");
                
                }

            }

            rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>> goal_handle){
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal cancel request...");
                return rclcpp_action::CancelResponse::ACCEPT;
            }

        public:
            explicit SimpleActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("simple_action_server", options){
                action_server = rclcpp_action::create_server<arduinobot_msgs::action::Fibonacci>(this, "fibonacci",
                    std::bind(&SimpleActionServer::handleGoal, this, _1, _2),
                    std::bind(&SimpleActionServer::handleCancel, this, _1),
                    std::bind(&SimpleActionServer::handleAccepted, this, _1));

                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Action Server is ready ...");
            }
    };
}


RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_action_server::SimpleActionServer)