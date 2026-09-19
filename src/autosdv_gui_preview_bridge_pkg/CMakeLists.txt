cmake_minimum_required(VERSION 3.8)
project(autosdv_gui_preview_bridge_pkg)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(preview_bridge_node src/preview_bridge_node.cpp)
ament_target_dependencies(
  preview_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_include_directories(preview_bridge_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(preview_bridge_node ${OpenCV_LIBRARIES})

install(TARGETS preview_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()
