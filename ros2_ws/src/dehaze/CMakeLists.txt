cmake_minimum_required(VERSION 3.8)
project(dehaze)

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

add_library(udcp_dehaze_component SHARED src/udcp_dehaze_node.cpp)
target_include_directories(udcp_dehaze_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(udcp_dehaze_component ${OpenCV_LIBS})
ament_target_dependencies(udcp_dehaze_component rclcpp rclcpp_components sensor_msgs)

add_executable(udcp_dehaze_check src/udcp_dehaze_node.cpp)
target_compile_definitions(udcp_dehaze_check PRIVATE UDCP_DEHAZE_SELF_CHECK)
target_include_directories(udcp_dehaze_check PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(udcp_dehaze_check ${OpenCV_LIBS})
ament_target_dependencies(udcp_dehaze_check rclcpp rclcpp_components sensor_msgs)

rclcpp_components_register_node(udcp_dehaze_component
  PLUGIN "dehaze::UDCPDehazeNode"
  EXECUTABLE udcp_dehaze_node
)

install(TARGETS udcp_dehaze_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
