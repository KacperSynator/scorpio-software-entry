cmake_minimum_required(VERSION 3.12)
project(virtual_dc_motor CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED  ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include/virtual_dc_motor
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(virtual_dc_motor src/virtual_dc_motor.cpp)
target_link_libraries(virtual_dc_motor ${catkin_LIBRARIES})

add_executable(velocity_sensor_node src/velocity_sensor_node.cpp)
target_link_libraries(velocity_sensor_node ${catkin_LIBRARIES})
