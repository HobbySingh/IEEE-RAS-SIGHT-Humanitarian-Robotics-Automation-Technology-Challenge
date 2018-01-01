cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  message_filters
  move_base_msgs
  actionlib
  angles
  laser_assembler
  control_msgs
  visualization_msgs 
  sensor_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    holonomic_control
    pid_controller
  CATKIN_DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(pid_controller src/pid_controller.cpp) 
#add_executable(pid_controller src/pid_controller.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES})
install(TARGETS pid_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(temp src/temp.cpp)
target_link_libraries(temp ${catkin_LIBRARIES})
install(TARGETS temp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(holonomic_control src/holonomic_control.cpp) 
#add_executable(holonomic_control src/holonomic_control.cpp)
target_link_libraries(holonomic_control ${catkin_LIBRARIES})
install(TARGETS holonomic_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)