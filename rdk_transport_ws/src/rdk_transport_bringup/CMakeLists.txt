cmake_minimum_required(VERSION 3.8)
project(rdk_transport_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch scripts docs config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
