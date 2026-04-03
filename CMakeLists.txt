cmake_minimum_required(VERSION 3.8)
project(ugv_peripherals)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -------------------------------------------------------
# dependencies
# -------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ugv_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ALSA REQUIRED)

add_library(lights_controller SHARED 
    src/lights_server.cpp)

target_include_directories(lights_controller PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_definitions(lights_controller PRIVATE 
    "UGV_PERIPHERALS_BUILDING_DLL")

ament_target_dependencies(lights_controller
    "ugv_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")

rclcpp_components_register_node(lights_controller PLUGIN "ugv_peripherals::LightsController" EXECUTABLE lights_server)

install(TARGETS
    lights_controller
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)


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

target_link_libraries(action_server
  asound
)

rclcpp_components_register_node(action_server PLUGIN "ugv_peripherals::SpeakerActionServer" EXECUTABLE speaker_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# END ACTION SERVER


# -------------------------------------------------------
# deterrence_event_trigger.cpp (action client)
# -------------------------------------------------------

# dependencies already included in section above via find_package

# allows node to be loaded as a component --> less overhead when sharing data between nodes
add_library(deterrence_event_trigger SHARED
  src/deterrence_event_trigger.cpp)

# helps deterrence_event_trigger find headers
target_include_directories(deterrence_event_trigger PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# note: the plugin is the C++ class name 
rclcpp_components_register_node(deterrence_event_trigger PLUGIN "ugv_peripherals::DeterrenceActionClient" EXECUTABLE deterrence_event_trigger)

# meaning: after building, copy the resulting files () to the proper install folders for libraries and executables, so they can be used by other packages or run at runtime.”
install(TARGETS deterrence_event_trigger
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_target_dependencies(deterrence_event_trigger
        "ugv_interfaces"
        "rclcpp"
        "rclcpp_action"
        "rclcpp_components"
        "sensor_msgs")


install(DIRECTORY resource/audio_lists_wav/
  DESTINATION share/${PROJECT_NAME}/resource/audio_lists_wav
  FILES_MATCHING PATTERN "*.wav")

ament_package()
