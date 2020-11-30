#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define PI 3.14159
#define RATE 100

//PARAMETERS------------------------------------------------
static const int nJoints = 12;
static const int nLegs = 4;


static const float bodyLength = 1;
static const float bodyWidth = .4;
static const float shoulderLength = .1;
static const float legLength = .4;
static const float calfLength = .35;
static const float weight = 100;

//
static const float omega = 0;
static const float phi = 0;
static const float psi = 0;
static const float xm = 0;
static const float ym = .5;
static const float zm = 0;


//joint names
std::vector<std::string> jointName= {"left_front_leg_shoulder_rotate", "right_front_leg_shoulder_rotate", "left_back_leg_shoulder_rotate", "right_back_leg_shoulder_rotate",
											"left_front_leg_shoulder_hinge", "right_front_leg_shoulder_hinge", "left_back_leg_shoulder_hinge", "right_back_leg_shoulder_hinge",
											"left_front_leg_elbow_hinge", "right_front_leg_elbow_hinge", "left_back_leg_elbow_hinge", "right_back_leg_elbow_hinge"};




sensor_msgs::JointState jointState;

void jointCallback(const sensor_msgs::JointState::ConstPtr &_js){

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

void freeze(sensor_msgs::JointState jointState, std::vector<ros::Publisher> jointPub){
		for(int i = 0; i < nJoints; i++){			
			
		//	//std::cout << "quadrupedal_test/" + initialState.name[i] + "_controller/command" << std::endl;

			std_msgs::Float64 msg;
			msg.data = jointState.position[i];
			//msg.data = 1.5;
			jointPub[i].publish(msg);
		}
}

void getThere(sensor_msgs::JointState jointState_, sensor_msgs::JointState objectiveState, std::vector<ros::Publisher> jointPub, int vel){

	int distPerLoop = vel/RATE;
	float dist;
	std_msgs::Float64 msg;

	for(int i = 0; i < nJoints; i++){
		if (jointState_.position[i] - objectiveState.position[i] <= -distPerLoop)
			jointState_.position[i] += distPerLoop;
		else if (jointState_.position[i] - objectiveState.position[i] >= distPerLoop)
			jointState_.position[i] -= distPerLoop;

		msg.data = jointState_.position[i];

		jointPub[i].publish(msg);
	}

}

sensor_msgs::JointState crouch(std::vector<std::string> jointName){
	sensor_msgs::JointState jointState_;

	jointState_.name.resize(nJoints);
	jointState_.position.resize(nJoints);

	jointState_.name = jointName;

	for(int i = 0; i < nJoints; i++){
		if(jointState_.name[i].find("_shoulder_hinge") != std::string::npos){

			jointState_.position[i] = 0;

		}else if(jointState_.name[i].find("_shoulder_rotate") != std::string::npos){
			if(jointState_.name[i].find("left") != std::string::npos)
				jointState_.position[i] = -.28;
			else
				jointState_.position[i] = .28;			
		}else if(jointState_.name[i].find("_leg_elbow") != std::string::npos){
			if(jointState_.name[i].find("left") != std::string::npos)
				jointState_.position[i] = .6;
			else
				jointState_.position[i] = -.6;
		}
	}

	return jointState_;
}

sensor_msgs::JointState standUp(std::vector<std::string> jointName){
	sensor_msgs::JointState jointState_;

	jointState_.name.resize(nJoints);
	jointState_.position.resize(nJoints);

	jointState_.name = jointName;

	for(int i = 0; i < nJoints; i++){
		jointState_.position[i] = 0;
	}

	return jointState_;
}

void printJointState(sensor_msgs::JointState jointState){
	if(jointState.header.seq != 0)
		for(int i = 0; i < nJoints; i++){
			std::cout << "-------" <<std::endl;
			std::cout << jointState.name[i] <<std::endl;
			std::cout << jointState.position[i] <<std::endl;
		}
}

void directCinematic(){

}

void inverseCinematic(){
	
}

std::vector<float> calculateTheta(std::string type, sensor_msgs::JointState jointState_){

	std::vector<float> theta(nLegs);

	for(int i = 0; i < nJoints; i++){
		if(jointState_.name[i].find(type) != std::string::npos){
			if(jointState_.name[i].find("left_front") != std::string::npos)

				theta[0] = jointState_.position[i];

			else if(jointState_.name[i].find("right_front") != std::string::npos)

				theta[1] = jointState_.position[i];

			else if(jointState_.name[i].find("left_back") != std::string::npos)

				theta[2] = jointState_.position[i];

			else

				theta[3] = jointState_.position[i];
		}
	}


	return theta;
}

std::vector<std::vector<float>> matrixMulti(std::vector<std::vector<float>> m1, std::vector<std::vector<float>> m2){
	std::vector<std::vector<float>> multi(4, std::vector<float>(4));

	for(int i = 0; i < nLegs; i++){
 		for(int j = 0; j < nLegs; j++){
 			multi[i][j] = 0;
 			for(int k = 0; k < nLegs; k++){
 				multi[i][j] += m1[i][k]*m2[k][j];
 			}
 		}
 	}

 	return multi;
}

std::vector<std::vector<float>> matrixSum(std::vector<std::vector<float>> m1, std::vector<std::vector<float>> m2){
	std::vector<std::vector<float>> sum(nLegs, std::vector<float>(nLegs));

	for(int i = 0; i < nLegs; i++)
 		for(int j = 0; j < nLegs; j++)
 			sum[i][j] = m1[i][j] + m2[i][j];

	return sum;
}


std::vector<std::vector<float>> createIdent(int n){
	std::vector<std::vector<float>> matrix(n, std::vector<float>(n, 0.0));

	for(int i = 0; i < n; i++)
		for(int j = 0; j < n; j++)
			if (i == j)
				matrix[i][j] = 0;

	return matrix;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "controllerTesting");
	ros::NodeHandle n;


	//waiting ros node got ready
	ros::Time last_ros_time_;
	bool wait = true;
	while(wait){
		last_ros_time_ = ros::Time::now();
		if (last_ros_time_.toSec() > 0)
			wait = false;
	}

	//ros::Time time = ros::Time::now();

	std::vector<ros::Publisher> jointPub(nJoints);

	jointState.name.resize(nJoints);
	jointState.position.resize(nJoints);

	std::sort(jointName.begin(), jointName.end());

	for(int i = 0; i < nJoints; i++)
		jointPub[i] = n.advertise<std_msgs::Float64>("/quadrupedal_test/" + jointName[i] + "_controller/command", 5);
	
	ros::Subscriber sub = n.subscribe("/quadrupedal_test/joint_states", 10, jointCallback);
	ros::Rate r(RATE);
	
	
	std::string robot_desc_string;
	n.param("robot_description", robot_desc_string, std::string());


	std::vector<float> cm(3);
	cm[0] = xm;
	cm[1] = ym;
	cm[2] = zm;

	//determinando ponto final
	std::vector<std::vector<float>> objPoint(4, std::vector<float>(3, 0.0));
	
	objPoint[0] = {0, 0, 0};
	objPoint[1]	= {0, 0, 0};
	objPoint[2]	= {0, 0, 0};
	objPoint[3]	= {0, 0, 0};

	std::vector<std::vector<float>> dx = objPoint;

	sensor_msgs::JointState crouchState = crouch(jointName);
	//calculation initiates------------------------------
	//body rotation matrix
	std::vector<std::vector<float>> Rx(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Ry(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Rz(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Rxyz(nLegs, std::vector<float>(nLegs));
	Rx = createIdent(nLegs);
	Rz = Ry = Rx;

	Rx[2][2] = Rx[3][3] = cos(omega);
	Rx[2][3] = -sin(omega);
	Rx[3][2] = sin(omega);

	Ry[1][1] = Ry[3][3] = cos(phi);
	Ry[1][1] = sin(phi);
 	Ry[3][3] = -sin(phi);
 	
 	Rxyz = matrixMulti(matrixMulti(Rx, Ry), Rz);
	 
	//center M transformation
 	std::vector<std::vector<float>> matrixAux(nLegs, std::vector<float>(nLegs));
 	matrixAux[0][3] = xm;
 	matrixAux[1][3] = ym;
 	matrixAux[2][3] = zm;
 	matrixAux[3][3] = 0;


	std::vector<std::vector<float>> Tm = matrixSum(matrixMulti(Rxyz, createIdent(nLegs)),matrixAux);

	//Transformation for each leg

	std::vector<std::vector<float>> right_back_leg_matrix(4, std::vector<float>(4));
		right_back_leg_matrix[0] = {0, 0, 1, -bodyLength/2};
		right_back_leg_matrix[1] = {0, 1, 0, 0};
		right_back_leg_matrix[2] = {-1, 0, 0, bodyWidth/2};
		right_back_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> right_front_leg_matrix(4, std::vector<float>(4));
		right_front_leg_matrix[0] = {0, 0, 1, bodyLength/2};
		right_front_leg_matrix[1] = {0, 1, 0, 0};
		right_front_leg_matrix[2] = {-1, 0, 0, bodyWidth/2};
		right_front_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> left_front_leg_matrix(4, std::vector<float>(4));
		left_front_leg_matrix[0] = {0, 0, -1, bodyLength/2};
		left_front_leg_matrix[1] = {0, 1, 0, 0};
		left_front_leg_matrix[2] = {1, 0, 0, -bodyWidth/2};
		left_front_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> left_back_leg_matrix(4, std::vector<float>(4));
		left_back_leg_matrix[0] = {0, 0, -1, -bodyLength/2};
		left_back_leg_matrix[1] = {0, 1, 0, 0};
		left_back_leg_matrix[2] = {1, 0, 0, -bodyWidth/2};
		left_back_leg_matrix[3] = {0, 0, 0, 1};

	std::vector<std::vector<float>> Trb = matrixMulti(Tm, right_back_leg_matrix);
	std::vector<std::vector<float>> Trf = matrixMulti(Tm, right_front_leg_matrix);
	std::vector<std::vector<float>> Tlb = matrixMulti(Tm, left_front_leg_matrix);
	std::vector<std::vector<float>> Tlf = matrixMulti(Tm, left_back_leg_matrix);

	

	while(ros::ok()){

		//initial leg angle
		std::vector<float> theta1 = calculateTheta("shoulder_hinge", jointState);
		std::vector<float> theta2 = calculateTheta("shoulder_rotate", jointState);
		std::vector<float> theta3 = calculateTheta("elbow_hinge", jointState);


		for(int i = 0; i < nJoints; i++){			
			

			std_msgs::Float64 msg;
			msg.data = crouchState.position[i];
			//msg.data = 1.5;
			jointPub[i].publish(msg);
		}



		ros::spinOnce();
	}

}