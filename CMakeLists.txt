cmake_minimum_required(VERSION 3.5)
project(sh_scc)

find_package(rclcpp REQUIRED)
find_package(sh_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sh_common_interfaces REQUIRED)
find_package(sh_scc_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(sh_color_peaks_calculator_node
    src/nodes/color_peaks_calculator_node.cpp
    src/lib/color_peaks_calculator_node.cpp
)
ament_target_dependencies(sh_color_peaks_calculator_node
    rclcpp
    cv_bridge
    sh_common
    sensor_msgs
    sh_common_interfaces
    sh_scc_interfaces
    OpenCV
)

add_executable(cv_bridge_node src/nodes/cv_bridge_node.cpp)
ament_target_dependencies(cv_bridge_node
    rclcpp
    sh_common
    cv_bridge
    image_transport
    sensor_msgs
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS sh_color_peaks_calculator_node cv_bridge_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_package()
