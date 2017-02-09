cmake_minimum_required(VERSION 2.8.3)
project(descartes_tutorials)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  trajectory_msgs
)

catkin_package()

#set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#baxter
 add_executable(${PROJECT_NAME}_tutorial1 src/tutorial1.cpp)

 target_link_libraries(${PROJECT_NAME}_tutorial1
   ${catkin_LIBRARIES}
 )

set_target_properties(${PROJECT_NAME}_tutorial1
  PROPERTIES OUTPUT_NAME tutorial1
  PREFIX "")

#motoman
add_executable(${PROJECT_NAME}_tutorial2 src/tutorial2.cpp)

 target_link_libraries(${PROJECT_NAME}_tutorial2
   ${catkin_LIBRARIES}
 )

set_target_properties(${PROJECT_NAME}_tutorial2
  PROPERTIES OUTPUT_NAME tutorial2
  PREFIX "")

#crustcrawler
add_executable(${PROJECT_NAME}_tutorial3 src/tutorial3.cpp)

 target_link_libraries(${PROJECT_NAME}_tutorial3
   ${catkin_LIBRARIES}
 )

set_target_properties(${PROJECT_NAME}_tutorial3
  PROPERTIES OUTPUT_NAME tutorial3
  PREFIX "")

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_tutorial1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_tutorial2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_tutorial3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
