cmake_minimum_required(VERSION 3.5)
project(franka_broadcasters)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(franka_example_controllers REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  external_torques_broadcaster_parameters
  src/external_torques_broadcaster.yaml
)

generate_parameter_library(
  external_wrench_broadcaster_parameters
  src/external_wrench_broadcaster.yaml
)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/external_torques_broadcaster.cpp
        src/external_wrench_broadcaster.cpp
)


include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
)
# Link parameter libraries to the main library
target_link_libraries(${PROJECT_NAME}
  external_torques_broadcaster_parameters
  external_wrench_broadcaster_parameters
)


ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        franka_semantic_components
        generate_parameter_library
)

pluginlib_export_plugin_description_file(
        controller_interface franka_broadcasters.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
)

ament_package()
