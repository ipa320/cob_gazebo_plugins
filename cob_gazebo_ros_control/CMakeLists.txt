cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo_ros_control)

find_package(catkin REQUIRED COMPONENTS controller_manager gazebo_ros_control gazebo_ros hardware_interface joint_limits_interface pluginlib roscpp transmission_interface urdf)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS controller_manager gazebo_ros_control gazebo_ros hardware_interface joint_limits_interface pluginlib roscpp transmission_interface urdf
  DEPENDS gazebo
  INCLUDE_DIRS include
  LIBRARIES hwi_switch_gazebo_ros_control hwi_switch_robot_hw_sim
)

### BUILD ###
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(hwi_switch_robot_hw_sim src/hwi_switch_robot_hw_sim.cpp)
target_link_libraries(hwi_switch_robot_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(hwi_switch_gazebo_ros_control src/hwi_switch_gazebo_ros_control_plugin.cpp)
target_link_libraries(hwi_switch_gazebo_ros_control hwi_switch_robot_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

### INSTALL ###
install(TARGETS hwi_switch_gazebo_ros_control hwi_switch_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES cob_gazebo_ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
