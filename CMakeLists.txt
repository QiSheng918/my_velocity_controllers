cmake_minimum_required(VERSION 2.8.3)
project(my_velocity_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  angles
  control_msgs
  control_toolbox
  controller_interface
  forward_command_controller
  realtime_tools
  urdf
)

catkin_package(
  CATKIN_DEPENDS
    angles
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    realtime_tools
    urdf
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)



include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  # src/joint_velocity_controller.cpp
  # src/joint_position_controller.cpp
  # src/joint_group_velocity_controller.cpp
  src/my_controller.cpp
  src/my_group_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})






