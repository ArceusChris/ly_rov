cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(v4l2_camera_component SHARED src/v4l2_camera_node.cpp)
target_include_directories(v4l2_camera_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(v4l2_camera_component ${OpenCV_LIBS})
ament_target_dependencies(v4l2_camera_component rclcpp rclcpp_components sensor_msgs)

rclcpp_components_register_node(v4l2_camera_component
  PLUGIN "camera_driver::V4L2CameraNode"
  EXECUTABLE v4l2_camera_node
)

install(TARGETS v4l2_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
