cmake_minimum_required(VERSION 3.8)
project(ugv_peripherals)

# compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ugv_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ALSA REQUIRED)

# lights server component
add_library(lights_server SHARED src/lights_controller.cpp)
ament_target_dependencies(lights_server
    "ugv_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
rclcpp_components_register_node(lights_server PLUGIN "ugv_peripherals::LightsController" EXECUTABLE lights_server_node)

# serial device component
add_library(serial_device SHARED src/serial_device_node.cpp)
ament_target_dependencies(serial_device 
    "ugv_interfaces"
    "rclcpp"
    "rclcpp_components")
rclcpp_components_register_node(serial_device PLUGIN "ugv_peripherals::SerialDeviceNode" EXECUTABLE serial_device_node)

# deterrence system component
add_library(deter_event_trig SHARED src/deterrence_event_trigger.cpp)
ament_target_dependencies(deter_event_trig
    "ugv_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
rclcpp_components_register_node(deter_event_trig PLUGIN "ugv_peripherals::DeterrenceActionClient" EXECUTABLE deter_event_trig_node)

# speaker server component
add_library(speaker_server SHARED src/speaker_action_server.cpp)
target_link_libraries(speaker_server ALSA::ALSA)
ament_target_dependencies(speaker_server
  "ugv_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(speaker_server PLUGIN "ugv_peripherals::SpeakerActionServer" EXECUTABLE speaker_server_node)

# install logic
set(all_targets lights_server serial_device deter_event_trig action_server action_client)
install(TARGETS ${all_targets}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

# audio files resource
install(DIRECTORY resource/audio_lists_wav/
  DESTINATION share/${PROJECT_NAME}/resource/audio_lists_wav
  FILES_MATCHING PATTERN "*.wav")

# testing and export 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
