cmake_minimum_required(VERSION 3.8)
project(rdk_transport_perception_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ai_detections_json_bridge src/ai_detections_json_bridge.cpp)
ament_target_dependencies(ai_detections_json_bridge
  ai_msgs
  rclcpp
  std_msgs
)

install(TARGETS ai_detections_json_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
