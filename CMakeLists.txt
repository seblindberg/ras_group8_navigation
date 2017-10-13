cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_navigation)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  move_base_msgs
  actionlib
)

catkin_package(
  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    move_base_msgs
    actionlib
    tf

# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#
# Main node
#

add_executable(${PROJECT_NAME}
  src/navigation_node.cpp
  src/Navigation.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#catkin_add_gtest(navigation_tests test/test_navigation.cpp src/Navigation.cpp)

# Make main executable
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#
# Client node
#

add_executable(${PROJECT_NAME}_client
  src/navigation_client_node.cpp
  #src/NavigationClient.cpp
)

add_dependencies(${PROJECT_NAME}_client
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_client
  ${catkin_LIBRARIES}
)

# Make client executable
install(
  TARGETS ${PROJECT_NAME}_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)