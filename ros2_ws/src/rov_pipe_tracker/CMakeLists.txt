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
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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
ament_target_dependencies(pipe_tracker_component rclcpp rclcpp_components rcl_interfaces ai_msgs std_srvs)
ament_target_dependencies(pipe_tracker_component std_msgs)

add_library(mavlink_manual_control_bridge_component SHARED
  src/mavlink_manual_control_bridge_node.cpp
)
add_dependencies(mavlink_manual_control_bridge_component generate_mavlink_headers)
target_include_directories(mavlink_manual_control_bridge_component PRIVATE ${MAVLINK_GEN_DIR})
ament_target_dependencies(
  mavlink_manual_control_bridge_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)

add_library(cv_pipe_segmenter_component SHARED src/cv_pipe_segmenter_node.cpp)
target_include_directories(cv_pipe_segmenter_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cv_pipe_segmenter_component ${OpenCV_LIBS})
ament_target_dependencies(
  cv_pipe_segmenter_component
  rclcpp
  rclcpp_components
  rcl_interfaces
  sensor_msgs
  ai_msgs
)

add_library(pipe_web_ui_component SHARED src/pipe_web_ui_node.cpp)
target_include_directories(pipe_web_ui_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pipe_web_ui_component ${OpenCV_LIBS})
ament_target_dependencies(
  pipe_web_ui_component
  rclcpp
  rclcpp_components
  sensor_msgs
  ai_msgs
  std_msgs
  std_srvs
)

add_library(manual_control_web_ui_component SHARED src/manual_control_web_ui_node.cpp)
ament_target_dependencies(
  manual_control_web_ui_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)

add_library(parameter_tuning_web_ui_component SHARED src/parameter_tuning_web_ui_node.cpp)
ament_target_dependencies(
  parameter_tuning_web_ui_component
  rclcpp
  rclcpp_components
  rcl_interfaces
)

rclcpp_components_register_node(pipe_tracker_component
  PLUGIN "rov_pipe_tracker::PipeTrackerNode"
  EXECUTABLE pipe_tracker_node
)

rclcpp_components_register_node(mavlink_manual_control_bridge_component
  PLUGIN "rov_pipe_tracker::MavlinkManualControlBridgeNode"
  EXECUTABLE mavlink_manual_control_bridge_node
)

rclcpp_components_register_node(cv_pipe_segmenter_component
  PLUGIN "rov_pipe_tracker::CvPipeSegmenterNode"
  EXECUTABLE cv_pipe_segmenter_node
)

rclcpp_components_register_node(pipe_web_ui_component
  PLUGIN "rov_pipe_tracker::PipeWebUiNode"
  EXECUTABLE pipe_web_ui_node
)

rclcpp_components_register_node(manual_control_web_ui_component
  PLUGIN "rov_pipe_tracker::ManualControlWebUiNode"
  EXECUTABLE manual_control_web_ui_node
)

rclcpp_components_register_node(parameter_tuning_web_ui_component
  PLUGIN "rov_pipe_tracker::ParameterTuningWebUiNode"
  EXECUTABLE parameter_tuning_web_ui_node
)

install(TARGETS
  pipe_tracker_component
  mavlink_manual_control_bridge_component
  cv_pipe_segmenter_component
  pipe_web_ui_component
  manual_control_web_ui_component
  parameter_tuning_web_ui_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/remote_cv_pipe_segmenter_server.py
  scripts/remote_mask_mjpeg_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
