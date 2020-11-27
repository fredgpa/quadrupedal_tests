#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


//namespace quadrupedal_test{
//}

static const std::vector<std::string> jointName= {"left_front_leg_shoulder_rotate", "right_front_leg_shoulder_rotate", "left_back_leg_shoulder_rotate", "right_back_leg_shoulder_rotate",
											"left_front_leg_shoulder_hinge", "right_front_leg_shoulder_hinge", "left_back_leg_shoulder_hinge", "right_back_leg_shoulder_hinge",
											"left_front_leg_elbow_hinge", "right_front_leg_elbow_hinge", "left_back_leg_elbow_hinge", "right_back_leg_elbow_hinge",
											"left_front_calf_elbow_hinge", "right_front_calf_elbow_hinge", "left_back_calf_elbow_hinge", "right_back_calf_elbow_hinge"};

static const int nJoints = 16;

ros::Publisher jointPub;
sensor_msgs::JointState jointState;

void jointCallback(const sensor_msgs::JointState::ConstPtr &_js){
	jointState.header.stamp = _js->header.stamp;
	jointState.name = _js->name;
	jointState.position = _js->position;

	jointPub.publish(jointState);
}

int main(int argc, char** argv){

	ros::init(argc, argv, "controllerTesting");
	ros::NodeHandle n;
	jointPub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Subscriber sub = n.subscribe("joint_states", 1, jointCallback);

	std::string robot_desc_string;
	n.param("robot_description", robot_desc_string, std::string());

	//for joint publishing
	

	std::vector<double> msg(4, 0.0);

	//std::vector

	while(ros::ok()){
		/*
		jointState.header.stamp = ros::Time::now();
		jointState.name.resize(nJoints);
		jointState.position.resize(nJoints);

		for(int i = 0; i < nJoints; i++){
			jointState.name[i] = jointName[i];

		}
		*/
	}

}