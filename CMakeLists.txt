cmake_minimum_required(VERSION 2.8.3)
project(fetch_open_auto_dock)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    angles
    cmake_modules
    fetch_auto_dock_msgs
    fetch_driver_msgs
    geometry_msgs
    nav_msgs
    roslib
    roscpp
    rospy
    std_msgs
    tf
)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    fetch_auto_dock_msgs
    fetch_driver_msgs
    geometry_msgs
    nav_msgs
    roslib
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(auto_dock
  src/auto_dock.cpp
  src/controller.cpp
  src/icp_2d.cpp
  src/laser_processor.cpp
  src/perception.cpp
  src/linear_pose_filter_2d.cpp
)
target_link_libraries(auto_dock
  ${catkin_LIBRARIES}
)
add_dependencies(auto_dock
  fetch_auto_dock_gencpp
  fetch_msgs_gencpp
  power_msgs_gencpp
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS auto_dock
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS
    scripts/dock_on_button.py
    scripts/dock_robot.py
    scripts/undock_on_button.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
