cmake_minimum_required(VERSION 3.8)
project(omni_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pi3hat_moteus_int_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Shared library
add_library(${PROJECT_NAME} SHARED src/omni_controller.cpp src/wheel_ik.cpp)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
    ${PROJECT_NAME}
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    pi3hat_moteus_int_msgs
    std_srvs
    geometry_msgs
    std_msgs)

pluginlib_export_plugin_description_file(controller_interface
                                         omni_controller.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)

# Tests
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_wheel_ik test/test_wheel_ik.cpp src/wheel_ik.cpp)
    target_include_directories(test_wheel_ik PRIVATE include)
endif()

ament_export_dependencies(
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    pi3hat_moteus_int_msgs
    std_srvs
    geometry_msgs
    std_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
