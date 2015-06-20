cmake_minimum_required(VERSION 2.8.3)
project(waypoint_list_maker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_srvs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(waypoint_list_maker
  src/waypoint_list_maker.cpp
)

add_executable(waypoint_list_maker_node src/waypoint_list_maker_node.cpp)

target_link_libraries(waypoint_list_maker
  ${catkin_LIBRARIES}
)

target_link_libraries(waypoint_list_maker_node
  waypoint_list_maker
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS waypoint_list_maker waypoint_list_maker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
