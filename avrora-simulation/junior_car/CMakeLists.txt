cmake_minimum_required(VERSION 2.8.3)

project(junior_car)

find_package(catkin REQUIRED roscpp rospy tf)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

find_package(roslaunch)


install(PROGRAMS
   nodes/ackermann_controller.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
