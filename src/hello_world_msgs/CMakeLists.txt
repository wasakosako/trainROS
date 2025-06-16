cmake_minimum_required(VERSION 3.5)
project(hello_world_msgs)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPLILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_genrate_interfaces(
  ${PROJECT_NAME}
  "srv/SetMessage.srv"
  "msg/Message.msg"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()