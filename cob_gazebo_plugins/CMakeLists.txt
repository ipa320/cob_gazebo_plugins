cmake_minimum_required(VERSION 3.0.2)
project(cob_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    gazebo_ros_mimic_joint
  CATKIN_DEPENDS
    roscpp
  DEPENDS
    gazebo
    SDF
)

## Plugins
add_library(gazebo_ros_mimic_joint src/gazebo_ros_mimic_joint.cpp)
target_link_libraries(gazebo_ros_mimic_joint ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  gazebo_ros_mimic_joint
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
