cmake_minimum_required(VERSION 3.5)

project(twist_mux)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(include)

add_executable(twist_mux
  src/twist_mux.cpp
  src/twist_mux_node.cpp
  src/twist_mux_diagnostics.cpp
  )
ament_target_dependencies(twist_mux
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_updater)

add_executable(twist_marker
  src/twist_marker.cpp
  )
ament_target_dependencies(twist_marker
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_updater)

install(TARGETS twist_mux twist_marker
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/joystick_relay.py
  DESTINATION bin)

foreach(dir launch config)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

if (BUILD_TESTING)
  #  find_package(ament_cmake_pytest REQUIRED)
  #  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test('twist_mux' test)
  #  find_package(launch_testing_ament_cmake)
  #  add_launch_test(test/test.launch.py)

  #  ament_add_pytest_test(twist_mux test)
  #find_package(ament_cmake_gtest REQUIRED)
  #ament_add_gtest(test_buffer test/test_buffer.cpp)
  #target_link_libraries(test_buffer ${PROJECT_NAME})
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
