cmake_minimum_required(VERSION 3.8)
project(launch_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(communication_mode test/test_communication_mode.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# rmw_profile is imported by the launch files here and by autosdv_gui_pkg.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../config/dds/fastdds_whitelist.xml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../config/dds/cyclonedds_ivn.xml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../config/zenoh/zenoh_session_ivn.json5"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../config/zenoh/zenoh_router_ivn.json5"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../config/dds/cyclonedds_bridge_local.xml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../config/zenoh/zenoh_bridge_camera_ivn.json5"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../config/zenoh/zenoh_bridge_control_ivn.json5"
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
