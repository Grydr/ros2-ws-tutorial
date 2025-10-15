cmake_minimum_required(VERSION 3.8)
project(simple_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(simple_interfaces REQUIRED)

add_executable(simple_action_server src/simpleAction.cpp)
ament_target_dependencies(simple_action_server PUBLIC rclcpp rclcpp_action simple_interfaces)

add_executable(simple_action_client src/simpleClient.cpp)
ament_target_dependencies(simple_action_client PUBLIC rclcpp rclcpp_action simple_interfaces)

install(
  TARGETS simple_action_server simple_action_client
  DESTINATION lib/${PROJECT_NAME}
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
