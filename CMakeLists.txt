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
