#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


//namespace quadrupedal_test{
//}

std::vector<std::string> jointName= {"left_front_leg_shoulder_rotate", "right_front_leg_shoulder_rotate", "left_back_leg_shoulder_rotate", "right_back_leg_shoulder_rotate",
											"left_front_leg_shoulder_hinge", "right_front_leg_shoulder_hinge", "left_back_leg_shoulder_hinge", "right_back_leg_shoulder_hinge",
											"left_front_leg_elbow_hinge", "right_front_leg_elbow_hinge", "left_back_leg_elbow_hinge", "right_back_leg_elbow_hinge",
											"left_front_calf_elbow_hinge", "right_front_calf_elbow_hinge", "left_back_calf_elbow_hinge", "right_back_calf_elbow_hinge"};

static const int nJoints = 16;


sensor_msgs::JointState jointState;
sensor_msgs::JointState initialState;

void jointCallback(const sensor_msgs::JointState::ConstPtr &_js){
	if(initialState.header.seq == 0){
		initialState.name.resize(nJoints);
		initialState.position.resize(nJoints);

		initialState.header.seq = _js->header.seq;
		initialState.header.stamp = _js->header.stamp;
		initialState.header.frame_id = _js->header.frame_id;

		for(size_t i = 0; i < nJoints; i++){
			//
			initialState.name[i] = _js->name[i];		
			initialState.position[i] = _js->position[i];
		}

	}else{
		jointState.name.resize(nJoints);
		jointState.position.resize(nJoints);

		jointState.header.seq = _js->header.seq;
		jointState.header.stamp = _js->header.stamp;
		jointState.header.frame_id = _js->header.frame_id;

		for(size_t i = 0; i < nJoints; i++){
			//
			jointState.name[i] = _js->name[i];		
			jointState.position[i] = _js->position[i];
		}
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "controllerTesting");
	ros::NodeHandle n;
	std::vector<ros::Publisher> jointPub(nJoints);

	std::sort(jointName.begin(), jointName.end());

	for(int i = 0; i < nJoints; i++)
		jointPub[i] = n.advertise<std_msgs::Float64>("/quadrupedal_test/" + jointName[i] + "_controller/command", 5);
	
	ros::Subscriber sub = n.subscribe("/quadrupedal_test/joint_states", 1, jointCallback);
	ros::Rate r(30);
	
	jointState.name.resize(nJoints);
	jointState.position.resize(nJoints);

	std::string robot_desc_string;
	n.param("robot_description", robot_desc_string, std::string());

	//for joint publishing
	// sensor_msgs::JointState initialState;
	// std::cout << initialState.header.seq << std::endl;
	//std::cout << "quadrupedal_test/" + initialState.name[i] + "_controller/command" << std::endl;
	//command to be publisherd
	std_msgs::Float64 msg;
	//std::vector<double> msg(4, 0.0);

	//std::vector

	while(ros::ok()){
		if(initialState.header.seq != 0){
			for(int i = 0; i < nJoints; i++){			
				
			//	//std::cout << "quadrupedal_test/" + initialState.name[i] + "_controller/command" << std::endl;

				msg.data = initialState.position[i];
				//msg.data = 1.5;
				jointPub[i].publish(msg);
			}
		}

		ros::spinOnce();
	}

}