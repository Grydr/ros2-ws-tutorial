cmake_minimum_required(VERSION 3.8)
project(simple_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simple_interfaces REQUIRED)

add_library(fib src/fib.cpp)
target_include_directories(fib 
  PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_executable(simple_pub src/pub.cpp)
target_link_libraries(simple_pub PUBLIC fib)
ament_target_dependencies(simple_pub PUBLIC rclcpp std_msgs)

add_executable(pub_with_param src/pubWithParam.cpp)
target_link_libraries(pub_with_param PUBLIC fib)
ament_target_dependencies(pub_with_param PUBLIC rclcpp std_msgs)

add_executable(simple_sub src/sub.cpp)
ament_target_dependencies(simple_sub PUBLIC rclcpp std_msgs)

add_executable(pub_custom_msg src/pubCustomMsg.cpp)
ament_target_dependencies(pub_custom_msg PUBLIC rclcpp simple_interfaces)

add_executable(sub_custom_msg src/subCustomMsg.cpp)
ament_target_dependencies(sub_custom_msg PUBLIC rclcpp simple_interfaces)

install(
  TARGETS 
  simple_pub pub_custom_msg simple_sub pub_custom_msg sub_custom_msg
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
