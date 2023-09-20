cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotinfo_msgs
  roscpp
  std_srvs
  cv_bridge
  image_transport
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gui
#  CATKIN_DEPENDS geometry_msgs nav_msgs robotinfo_msgs roscpp std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare the library for robot_gui
add_library(robot_gui_lib src/robot_gui.cpp)
target_link_libraries(robot_gui_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(robot_gui_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare the main node and link it with robot_gui_lib
add_executable(robot_gui_node src/robot_gui_node.cpp)
target_link_libraries(robot_gui_node robot_gui_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(robot_gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# Mark executables for installation
install(TARGETS robot_gui_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
