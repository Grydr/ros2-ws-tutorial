cmake_minimum_required(VERSION 3.8)
project(simple_servcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(simple_interfaces REQUIRED)

add_executable(simple_service src/simpleService.cpp)
ament_target_dependencies(simple_service rclcpp example_interfaces)

add_executable(simple_client src/simpleClient.cpp)
ament_target_dependencies(simple_client rclcpp example_interfaces)

# add_executable(simple_async_client src/simpleAsyncClient.cpp)
# ament_target_dependencies(simple_async_client rclcpp example_interfaces)

add_executable(service_custom_srv src/serviceCustomSrv.cpp)
ament_target_dependencies(service_custom_srv rclcpp simple_interfaces)

add_executable(client_custom_srv src/clientCustomSrv.cpp)
ament_target_dependencies(client_custom_srv rclcpp simple_interfaces)

install(
  TARGETS 
  simple_service 
  simple_client 
  # simple_async_client
  service_custom_srv
  client_custom_srv
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
