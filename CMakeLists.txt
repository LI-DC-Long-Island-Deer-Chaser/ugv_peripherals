cmake_minimum_required(VERSION 3.8)
project(ugv_peripherals)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ugv_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(lights_server SHARED 
    src/lights_controller.cpp)

target_include_directories(lights_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_definitions(lights_server PRIVATE 
    "UGV_PERIPHERALS_BUILDING_DLL")

ament_target_dependencies(lights_server
    "ugv_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")

rclcpp_components_register_node(lights_server PLUGIN "ugv_peripherals::LightsController" EXECUTABLE lights_controller)


add_library(lights_test SHARED 
    src/blink_test.cpp)

target_include_directories(lights_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_definitions(lights_test PRIVATE 
    "UGV_PERIPHERALS_BUILDING_DLL")

ament_target_dependencies(lights_test
    "ugv_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")

rclcpp_components_register_node(lights_test PLUGIN "ugv_peripherals::LightsTestClient" EXECUTABLE lights_test_client)

install(TARGETS
    lights_server
    lights_test
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

find_package(sensor_msgs REQUIRED)

# BEGIN ACTION SERVER
add_library(action_server SHARED
  src/speaker_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# FOR WINDOWS. NOT NECESSARY, CONSIDER REMOVING?
target_compile_definitions(action_server
  PRIVATE "ACTION_SPEAKERS_CPP_BUILDING_DLL")

ament_target_dependencies(action_server
  "ugv_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "sensor_msgs")

rclcpp_components_register_node(action_server PLUGIN "ugv_peripherals::SpeakerActionServer" EXECUTABLE speaker_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# END ACTION SERVER

# BEGIN ACTION CLIENT
add_library(action_client SHARED
  src/speaker_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Were not on windows
#target_compile_definitions(action_client
  #PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "ugv_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "sensor_msgs")

rclcpp_components_register_node(action_client PLUGIN "ugv_peripherals::SpeakerActionClient" EXECUTABLE speaker_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# END ACTION CLIENT

# I forgot I needed the resource folder
install(DIRECTORY resource/audio_lists_wav/
  DESTINATION share/${PROJECT_NAME}/resource/audio_lists_wav
  FILES_MATCHING PATTERN "*.wav")

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
