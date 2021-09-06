cmake_minimum_required(VERSION 3.0.2)
project(quadrupedal_tests)

add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	tf
	kdl_parser
	trac_ik_lib
	cv_bridge
	nav_msgs
	controller_manager
  	joint_state_controller
  	robot_state_publisher
)
find_package(OpenCV REQUIRED)

find_package(Armadillo REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})



catkin_package(
   INCLUDE_DIRS
#  LIBRARIES quadrupedal_tests
   CATKIN_DEPENDS roscpp
   				  sensor_msgs
   				  std_msgs
   				  cv_bridge
#  DEPENDS system_lib
)


include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIR} 
	${GAZEBO_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${ARMADILLO_INCLUDE_DIR}
)


add_executable(set_transforms src/set_transforms.cpp)
add_executable(test_node src/test_node.cpp)
add_executable(RobotLeg src/RobotLeg.cpp)
add_executable(controllerTesting src/controllerTesting.cpp)
add_executable(legsFrequency src/legsFrequency.cpp)
add_executable(quadrupedal_control_rviz src/quadrupedal_control_rviz.cpp)
target_link_libraries(set_transforms ${catkin_LIBRARIES})
target_link_libraries(test_node ${catkin_LIBRARIES})
target_link_libraries(RobotLeg ${catkin_LIBRARIES})
target_link_libraries(controllerTesting ${catkin_LIBRARIES} ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})
target_link_libraries(legsFrequency ${catkin_LIBRARIES})
target_link_libraries(quadrupedal_control_rviz ${catkin_LIBRARIES} ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})
