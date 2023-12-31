cmake_minimum_required(VERSION 3.8)
project(arduino_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(arduinobot_msgs REQUIRED)

# cpp executables
add_executable(angle_conversion_services src/angle_conversion_services.cpp)
ament_target_dependencies(angle_conversion_services rclcpp arduinobot_msgs tf2)

#install cpp executables
install(
  TARGETS
    angle_conversion_services
  DESTINATION
    lib/${PROJECT_NAME}
)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables
install(
  PROGRAMS
    ${PROJECT_NAME}/angle_conversion_service.py
  DESTINATION 
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
