cmake_minimum_required(VERSION 3.16)

project(twist_mux)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_executable(twist_mux
  src/twist_mux.cpp
  src/twist_mux_node.cpp
  src/twist_mux_diagnostics.cpp
)
target_include_directories(twist_mux PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(twist_mux
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${diagnostic_updater_TARGETS}
)

add_executable(twist_marker
  src/twist_marker.cpp
)
target_link_libraries(twist_marker
  ${rclcpp_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(
  TARGETS twist_mux twist_marker
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/joystick_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

foreach(dir launch config)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_joystick_relay.py)
endif()

ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_updater
)

ament_package()
