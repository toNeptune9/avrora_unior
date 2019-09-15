cmake_minimum_required(VERSION 2.8.3)
project(junior_2dnav)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf 
  geometry_msgs
  nav_msgs
  tf2 
  tf2_ros
)

catkin_package()


find_package(roslaunch)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rectificated_laser src/rect_laser.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(rectificated_laser ${catkin_LIBRARIES})
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})


