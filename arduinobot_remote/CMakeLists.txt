cmake_minimum_required(VERSION 3.8)
project(arduinobot_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(arduinobot_msgs REQUIRED)

add_library(arduinobot_task_server SHARED src/arduinobot_task_server.cpp)
target_include_directories(
  arduinobot_task_server
  PRIVATE "ARDUINOBOT_TASK_SERVER_CPP_BUILDING_DLL"
)
ament_target_dependencies(
  arduinobot_task_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "arduinobot_msgs"
)

rclcpp_components_register_node(
  arduinobot_task_server
  PLUGIN "arduinobot_remote::ArduinobotTaskServer"
  EXECUTABLE arduinobot_task_server_node
)

install(
  TARGETS 
    arduinobot_task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
