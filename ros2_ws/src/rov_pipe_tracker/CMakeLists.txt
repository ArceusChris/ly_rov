cmake_minimum_required(VERSION 3.8)
project(rov_pipe_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)

set(MAVLINK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../mavlink" CACHE PATH "mavlink submodule path")
set(MAVLINK_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/mavlink_generated/include/mavlink")
set(MAVLINK_PIP_DIR "${CMAKE_CURRENT_BINARY_DIR}/mavlink_pip")
set(MAVLINK_HEADER "${MAVLINK_GEN_DIR}/ardupilotmega/mavlink.h")

add_custom_command(
  OUTPUT ${MAVLINK_HEADER}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${MAVLINK_GEN_DIR}
  COMMAND ${Python3_EXECUTABLE}
          -m pip install
          -r ${MAVLINK_SOURCE_DIR}/pymavlink/requirements.txt
          --upgrade
          -t ${MAVLINK_PIP_DIR}
  COMMAND ${CMAKE_COMMAND} -E env
          "PYTHONPATH=${MAVLINK_SOURCE_DIR}/pymavlink:${MAVLINK_PIP_DIR}"
          ${Python3_EXECUTABLE}
          -m pymavlink.tools.mavgen
          --lang=C
          --wire-protocol=2.0
          --output ${MAVLINK_GEN_DIR}
          ${MAVLINK_SOURCE_DIR}/message_definitions/v1.0/ardupilotmega.xml
  COMMAND ${CMAKE_COMMAND} -E touch ${MAVLINK_HEADER}
  WORKING_DIRECTORY ${MAVLINK_SOURCE_DIR}
  DEPENDS
    ${MAVLINK_SOURCE_DIR}/message_definitions/v1.0/ardupilotmega.xml
)
add_custom_target(generate_mavlink_headers DEPENDS ${MAVLINK_HEADER})

add_library(pipe_tracker_component SHARED src/pipe_tracker_node.cpp)
add_dependencies(pipe_tracker_component generate_mavlink_headers)
target_include_directories(pipe_tracker_component PRIVATE ${MAVLINK_GEN_DIR})
ament_target_dependencies(pipe_tracker_component rclcpp rclcpp_components ai_msgs)

add_library(cv_pipe_segmenter_component SHARED src/cv_pipe_segmenter_node.cpp)
target_include_directories(cv_pipe_segmenter_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cv_pipe_segmenter_component ${OpenCV_LIBS})
ament_target_dependencies(cv_pipe_segmenter_component rclcpp rclcpp_components sensor_msgs ai_msgs)

rclcpp_components_register_node(pipe_tracker_component
  PLUGIN "rov_pipe_tracker::PipeTrackerNode"
  EXECUTABLE pipe_tracker_node
)

rclcpp_components_register_node(cv_pipe_segmenter_component
  PLUGIN "rov_pipe_tracker::CvPipeSegmenterNode"
  EXECUTABLE cv_pipe_segmenter_node
)

install(TARGETS pipe_tracker_component cv_pipe_segmenter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
