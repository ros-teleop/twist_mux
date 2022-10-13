cmake_minimum_required(VERSION 3.0.2)
project(twist_mux)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      std_msgs
      geometry_msgs
      visualization_msgs
      diagnostic_updater)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      roscpp
      std_msgs
      geometry_msgs
      visualization_msgs
      diagnostic_updater)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(twist_mux
    src/twist_mux.cpp
    src/twist_mux_node.cpp
    src/twist_mux_diagnostics.cpp)
target_link_libraries(twist_mux ${catkin_LIBRARIES})

add_executable(twist_marker
    src/twist_marker.cpp)
target_link_libraries(twist_marker ${catkin_LIBRARIES})

install(TARGETS twist_mux twist_marker
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/joystick_relay.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest(test/system.test)
endif()
