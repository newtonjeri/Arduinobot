#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


// Function declaration
void move_robot(const std::shared_ptr<rclcpp::Node>);


int main(int argc, char**argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
    move_robot(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// Function definition
void move_robot(const std::shared_ptr<rclcpp::Node> node){
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm_group");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper_group");

    std::vector<double> arm_joint_goal {1.5, 1.0, 1.0};
    std::vector<double> gripper_joint_goal {-0.70, 0.70};

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

    if (!arm_within_bounds | !gripper_within_bounds) {
        RCLCPP_ERROR(node->get_logger(), "Joint goal is not within limits");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (arm_plan_success && gripper_plan_success){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning was successful, ready to move :)");
        arm_move_group.move();
        gripper_move_group.move();
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planning was not successful, robot will not move :(");
        return;
    }

}