cmake_minimum_required(VERSION 3.8)
project(ros2_network_gateway)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Set C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED libzstd)

add_executable(network_gateway
        src/network_gateway.cpp
        src/subscription_manager.cpp
        src/network_interfaces/udp_interface.cpp
)
target_include_directories(network_gateway PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(network_gateway ${ZSTD_LIBRARIES})

ament_target_dependencies(network_gateway
        rclcpp
        rosidl_typesupport_cpp
        rosidl_typesupport_introspection_cpp
        rosidl_runtime_c)


install(TARGETS network_gateway
        DESTINATION lib/${PROJECT_NAME})

# Testing configuration
include(CTest)
if (BUILD_TESTING)
    # Simple smoke test to demonstrate CTest wiring without adding new sources
    add_test(NAME smoke COMMAND ${CMAKE_COMMAND} -E echo "smoke-ok")
endif ()

ament_package()
