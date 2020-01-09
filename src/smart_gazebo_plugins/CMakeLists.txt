cmake_minimum_required(VERSION 2.8.3)
project(smart_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(${PROJECT_NAME}_CATKIN_DEPS
    gazebo_ros
    smart_msgs)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(all_wheel_steering_plugin src/all_wheel_steering_plugin.cpp)
target_link_libraries(all_wheel_steering_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


