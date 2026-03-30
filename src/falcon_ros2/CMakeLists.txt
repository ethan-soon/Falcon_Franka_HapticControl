cmake_minimum_required(VERSION 3.5)
project(falcon_ros2)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Paths to the pre-built libnifalcon libraries.
# Override at build time with:
#   colcon build --cmake-args -DNIFALCON_ROOT=/path/to/NovintFalcon/EXTERNAL
# Defaults to ~/NovintFalcon/EXTERNAL (matching the original Simulink kit layout).
if(NOT DEFINED NIFALCON_ROOT)
    set(NIFALCON_ROOT "$ENV{HOME}/NovintFalcon/EXTERNAL")
endif()

set(NIFALCON_INCLUDE_DIR ${NIFALCON_ROOT}/include)
set(NIFALCON_LIB_DIR     ${NIFALCON_ROOT}/lib)

add_executable(falcon_node src/falcon_node.cpp)

target_include_directories(falcon_node PRIVATE ${NIFALCON_INCLUDE_DIR})

# Link against the same libs the Simulink build script used, plus libusb
target_link_libraries(falcon_node
    ${NIFALCON_LIB_DIR}/libnifalcon.so
    ${NIFALCON_LIB_DIR}/libnifalcon_cli_base.so
    usb-1.0
)

ament_target_dependencies(falcon_node
    rclcpp
    geometry_msgs
    std_msgs
)

install(TARGETS falcon_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
