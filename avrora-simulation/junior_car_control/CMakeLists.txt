cmake_minimum_required(VERSION 2.8.3)
project(junior_car_control)

find_package(catkin REQUIRED COMPONENTS

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES junior_car_control
#  CATKIN_DEPENDS gazebo_ros ros_control
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)