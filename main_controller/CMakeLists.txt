cmake_minimum_required(VERSION 2.8.3)
project(main_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controller
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    holonomic_control
    pid_controller
  CATKIN_DEPENDS 
    holonomic_control
  	pid_controller
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
#
add_executable(main_rover_control src/main_rover_control.cpp)
target_link_libraries(main_rover_control ${catkin_LIBRARIES} pid_controller)
install(TARGETS main_rover_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance ${catkin_LIBRARIES} pid_controller)
install(TARGETS obstacle_avoidance
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_executable(mine_detection src/mine_detection.cpp)
target_link_libraries(mine_detection ${catkin_LIBRARIES} pid_controller)
install(TARGETS mine_detection
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
