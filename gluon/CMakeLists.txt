cmake_minimum_required(VERSION 2.8.3)

project(gluon)

if( "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
	set(BUILD_PLATFORM
		linux_x86_64)
else()
	set(BUILD_PLATFORM
		windows_x64)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  message_generation
  roslib
  tf_conversions
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/include
	${catkin_INCLUDE_DIRS}
)

generate_messages(
		DEPENDENCIES
		std_msgs
		geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(${PROJECT_NAME}_node src/gluonControl.cpp) 

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node 
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/libActuatorController.so
)

