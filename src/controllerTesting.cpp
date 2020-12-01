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

std::vector<float> cross(std::vector<float> v1, std::vector<float> v2){
	std::vector<std::vector<float>> matrix(3,std::vector<float>(3));
	std::vector<float> vector(3);
	matrix[0] = {1, 1, 1};
	matrix[1] = v1;
	matrix[2] = v2;

	vector = {(matrix[1][1]*matrix[2][2] - matrix[1][2]*matrix[2][1]), (matrix[1][2]*matrix[2][1] - matrix[1][0]*matrix[2][2]), (matrix[1][0]*matrix[2][1] - matrix[1][1]*matrix[2][0])};

	return vector;
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

	for(int i = 0; i < m1[0].size(); i++){
 		for(int j = 0; j < m2.size(); j++){
 			multi[i][j] = 0;
 			for(int k = 0; k < nLegs; k++){
 				multi[i][j] += m1[i][k]*m2[k][j];
 			}
 		}
 	}

 	return multi;
}

std::vector<std::vector<float>> matrixArrayMulti(std::vector<std::vector<float>> matrix, std::vector<float> array){
	std::vector<std::vector<float>> multi(4, std::vector<float>(4));

	for(int i = 0; i < matrix[0].size(); i++){
 		for(int j = 0; j < array.size(); j++){
 			multi[i][j] = 0;
 			for(int k = 0; k < nLegs; k++){
 				multi[i][j] += matrix[i][k]*array[k];
 			}
 		}
 	}

 	return multi;
}

std::vector<std::vector<float>> matrixSumSub(std::vector<std::vector<float>> m1, std::vector<std::vector<float>> m2, bool aux){
	std::vector<std::vector<float>> matrix(nLegs, std::vector<float>(nLegs));

	for(int i = 0; i < nLegs; i++)
 		for(int j = 0; j < nLegs; j++){
 			if (aux)
 				matrix[i][j] = m1[i][j] + m2[i][j];
 			else
 				matrix[i][j] = m1[i][j] - m2[i][j];
 		}

	return matrix;
}

std::vector<std::vector<float>> matrixTransp(std::vector<std::vector<float>> a){
	std::vector<std::vector<float>> matrix(a.size(), std::vector<float>(a[0].size()));

	for (int i = 0; i < a[0].size(); i++)
		for (int j = 0; j < a.size(); j++)
			matrix[j][i] = a[i][j];

	return matrix;
}


std::vector<std::vector<float>> createIdent(int n){
	std::vector<std::vector<float>> matrix(n, std::vector<float>(n, 0.0));

	for(int i = 0; i < n; i++)
		for(int j = 0; j < n; j++)
			if (i == j)
				matrix[i][j] = 1;

	return matrix;
}

std::vector<std::vector<float>> Trans(float a, float b, float c, float d){
	//DH Homogeneous Transformation Matrix - Lucas Rigobello
	std::vector<std::vector<float>> T(4, std::vector<float>(4));

	T[0] = {cos(d), -sin(d)*round(cos(b)), sin(d)*sin(b), a*cos(d)};
	T[1] = {sin(d), cos(d)*round(cos(b)), -cos(d)*sin(b), a*sin(d)};
	T[2] = {0, sin(b), round(cos(b)), c};
	T[3] = {0, 0, 0, 1};

	return T;
}

std::vector<std::vector<float>> simplerMatrix(std::vector<std::vector<std::vector<float>>> a){
	int newSize = 0;
	for (int i = 0; i < a[0].size(); i++){
		newSize += a[0][i].size();
	}

	std::vector<std::vector<float>> matrix(a.size(), std::vector<float>(newSize));

	for (int i = 0; i < a.size(); i++)
		for(int j = 0; j < a[i].size(); j++)
			for(int k = 0; k < a[i][j].size(); k++)
				matrix[i][j+k] = a[i][j][k];

	return matrix;
}

float sum(std::vector<float> array){
	float result = 0;
	for (int i = 0; i < array.size(); i++){
		result += array[i];
	}

	return result;
}

std::vector<float> elementWiseMultiplication(std::vector<float> a, std::vector<float> b){
	std::vector<float> array;

	for(int i = 0; i < a.size(); i++)
		array[i] = a[i] * b[i];

	return array;
}

void forwardCinematic(){

	std::vector<float> theta1 = {0, 0, 0, 0};
	std::vector<float> theta2 = {-PI/3, -PI/3, PI/3, PI/3};
	std::vector<float> theta3 = {PI/6, PI/6, -PI/6, -PI/6};
	//initial leg angle
	// std::vector<float> theta1 = calculateTheta("shoulder_hinge", jointState);
	// std::vector<float> theta2 = calculateTheta("shoulder_rotate", jointState);
	// std::vector<float> theta3 = calculateTheta("elbow_hinge", jointState);
	

	std::vector<float> cm(3);
	cm[0] = xm;
	cm[1] = ym;
	cm[2] = zm;
	

	//determinando ponto final
	std::vector<std::vector<float>> objPoint(4, std::vector<float>(3, 0.0));
	
	objPoint[0] = {-.45, 0, .2};
	objPoint[1]	= {.65, 0, .2};
	objPoint[2]	= {.65, 0, -.2};
	objPoint[3]	= {-.45, 0, -.2};

	std::vector<std::vector<float>> dx = objPoint;

	
	//calculation initiates------------------------------
	//body rotation matrix
	std::vector<std::vector<float>> Rx(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Ry(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Rz(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Rxyz(nLegs, std::vector<float>(nLegs));

	Rx = createIdent(nLegs);
	Rz = Ry = Rx;

	Rx[1][1] = Rx[2][2] = cos(omega);
	Rx[1][2] = -sin(omega);
	Rx[2][1] = sin(omega);

	Ry[0][0] = Ry[2][2] = cos(phi);
	Ry[0][2] = sin(phi);
 	Ry[2][0] = -sin(phi);
 	
 	Rxyz = matrixMulti(matrixMulti(Rx, Ry), Rz);
	 
	//center M transformation
 	std::vector<std::vector<float>> matrixAux(nLegs, std::vector<float>(nLegs));
 	matrixAux[0][3] = xm;
 	matrixAux[1][3] = ym;
 	matrixAux[2][3] = zm;
 	matrixAux[3][3] = 0;

	std::vector<std::vector<float>> Tm = matrixSumSub(matrixMulti(Rxyz, createIdent(nLegs)),matrixAux, true);

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
	std::vector<std::vector<float>> Tlb = matrixMulti(Tm, left_back_leg_matrix);
	std::vector<std::vector<float>> Tlf = matrixMulti(Tm, left_front_leg_matrix);

	std::vector<std::vector<std::vector<float>>> P04 = {Trb, Trf, Tlf, Tlb};

	//position shoulder
	std::vector<float> pointC1 = {P04[0][0][3], P04[0][1][3], P04[0][2][3]};
	std::vector<float> pointC2 = {P04[1][0][3], P04[1][1][3], P04[1][2][3]};
	std::vector<float> pointC3 = {P04[2][0][3], P04[2][1][3], P04[2][2][3]};
	std::vector<float> pointC4 = {P04[3][0][3], P04[3][1][3], P04[3][2][3]};
	std::vector<std::vector<float>> pointC(3, std::vector<float>(4));
	pointC[0] = {pointC1[0], pointC2[0], pointC3[0], pointC4[0]};
	pointC[1] = {pointC1[1], pointC2[1], pointC3[1], pointC4[1]};
	pointC[2] = {pointC1[2], pointC2[2], pointC3[2], pointC4[2]};

	//Torque declaration
	std::vector<std::vector<std::vector<float>>> Torques(4);

	std::vector<std::vector<float>> Patas(4);

	for(int i = 0; i < 4; i++){

		float t1 = theta1[i];
		float t2 = theta2[i];
		float t3 = theta3[i];

		//inserting convertion parameters (a alpha d theta)
		std::vector<std::vector<float>> A01 = Trans(-shoulderLength, 0, 0, t1);
		std::vector<std::vector<float>> A02 = Trans(0, PI/2, 0, -PI/2);
		std::vector<std::vector<float>> A1 = matrixMulti(A01, A02);
		std::vector<std::vector<float>> A2 = Trans(legLength, 0, 0, t2);
		std::vector<std::vector<float>> A3 = Trans(calfLength, 0, 0, t3);
		A1 = matrixMulti(P04[i], A1);

		std::vector<std::vector<float>> T2 = matrixMulti(A1, A2);
		std::vector<std::vector<float>> T3 = matrixMulti(T2, A3);


		//creating zi
		std::vector<float> z0 = {P04[i][0][2], P04[i][1][2], P04[i][2][2]};
		std::vector<float> z1 = {A1[0][2], A1[1][2], A1[2][2]};
		std::vector<float> z2 = {T2[0][2], T2[1][2], T2[2][2]};
		std::vector<float> z3 = {T3[0][2], T3[1][2], T3[2][2]};

		

		//creating pi
		std::vector<float> p0 = {P04[i][0][3], P04[i][1][3], P04[i][2][3]};
		std::vector<float> p1 = {A1[0][3], A1[1][3], A1[2][3]};
		std::vector<float> p2 = {T2[0][3], T2[1][3], T2[2][3]};
		std::vector<float> P = {T3[0][3], T3[1][3], T3[2][3]};

		//Jacobian matrix
		std::vector<std::vector<float>> J(6, std::vector<float>(3));
		std::vector<float> aux1 = cross(z0, {P[0]-p0[0],P[1]-p0[1],P[2]-p0[2]});
		std::vector<float> aux2 = cross(z1, {P[0]-p1[0],P[1]-p1[1],P[2]-p1[2]});
		std::vector<float> aux3 = cross(z2, {P[0]-p2[0],P[1]-p2[1],P[2]-p2[2]});
		J[0] = {aux1[0], aux2[0], aux3[0]};
		J[1] = {aux1[1], aux2[1], aux3[1]};
		J[2] = {aux1[2], aux2[2], aux3[2]};
		J[3] = {z0[0], z1[0], z2[0]};
		J[4] = {z0[1], z1[1], z2[1]};
		J[5] = {z0[2], z1[2], z2[2]};


		std::vector<float> array = {0, weight/4, 0, 0, 0, 0};
		Torques[i] = matrixArrayMulti(matrixTransp(J), array);

		Patas[i] = P;
	}

	std::vector<std::vector<float>> newTorques = simplerMatrix(Torques);	
}

void inverseCinematic(){

	std::vector<float> theta1 = {0, 0, 0, 0};
	std::vector<float> theta2 = {-PI/3, -PI/3, PI/3, PI/3};
	std::vector<float> theta3 = {PI/6, PI/6, -PI/6, -PI/6};
	//initial leg angle
	// std::vector<float> theta1 = calculateTheta("shoulder_hinge", jointState);
	// std::vector<float> theta2 = calculateTheta("shoulder_rotate", jointState);
	// std::vector<float> theta3 = calculateTheta("elbow_hinge", jointState);
	

	std::vector<float> cm(3);
	cm[0] = xm;
	cm[1] = ym;
	cm[2] = zm;
	

	//determinando ponto final
	std::vector<std::vector<float>> objPoint(4, std::vector<float>(3, 0.0));
	
	objPoint[0] = {-.45, 0, .2};
	objPoint[1]	= {.65, 0, .2};
	objPoint[2]	= {.65, 0, -.2};
	objPoint[3]	= {-.45, 0, -.2};

	std::vector<std::vector<float>> dx = objPoint;

	
	//calculation initiates------------------------------
	//body rotation matrix
	std::vector<std::vector<float>> Rx(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Ry(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Rz(nLegs, std::vector<float>(nLegs));
	std::vector<std::vector<float>> Rxyz(nLegs, std::vector<float>(nLegs));

	Rx = createIdent(nLegs);
	Rz = Ry = Rx;

	Rx[1][1] = Rx[2][2] = cos(omega);
	Rx[1][2] = -sin(omega);
	Rx[2][1] = sin(omega);

	Ry[0][0] = Ry[2][2] = cos(phi);
	Ry[0][2] = sin(phi);
 	Ry[2][0] = -sin(phi);
 	
 	Rxyz = matrixMulti(matrixMulti(Rx, Ry), Rz);
	 
	//center M transformation
 	std::vector<std::vector<float>> matrixAux(nLegs, std::vector<float>(nLegs));
 	matrixAux[0][3] = xm;
 	matrixAux[1][3] = ym;
 	matrixAux[2][3] = zm;
 	matrixAux[3][3] = 0;

	std::vector<std::vector<float>> Tm = matrixSumSub(matrixMulti(Rxyz, createIdent(nLegs)),matrixAux, true);

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
	std::vector<std::vector<float>> Tlb = matrixMulti(Tm, left_back_leg_matrix);
	std::vector<std::vector<float>> Tlf = matrixMulti(Tm, left_front_leg_matrix);

	std::vector<std::vector<std::vector<float>>> P04 = {Trb, Trf, Tlf, Tlb};

	//position shoulder
	std::vector<float> pointC1 = {P04[0][0][3], P04[0][1][3], P04[0][2][3]};
	std::vector<float> pointC2 = {P04[1][0][3], P04[1][1][3], P04[1][2][3]};
	std::vector<float> pointC3 = {P04[2][0][3], P04[2][1][3], P04[2][2][3]};
	std::vector<float> pointC4 = {P04[3][0][3], P04[3][1][3], P04[3][2][3]};
	std::vector<std::vector<float>> pointC(3, std::vector<float>(4));
	pointC[0] = {pointC1[0], pointC2[0], pointC3[0], pointC4[0]};
	pointC[1] = {pointC1[1], pointC2[1], pointC3[1], pointC4[1]};
	pointC[2] = {pointC1[2], pointC2[2], pointC3[2], pointC4[2]};

	//Torque declaration
	std::vector<std::vector<std::vector<float>>> Torques(4);

	std::vector<std::vector<float>> Patas(4);

	for(int i = 0; i < nLegs; i++){

		float t1 = theta1[i];
		float t2 = theta2[i];
		float t3 = theta3[i];

		while(sum(elementWiseMultiplication(dx[i],dx[i])) >= .001){

			std::vector<std::vector<float>> A01 = Trans(-shoulderLength, 0, 0, t1);
			std::vector<std::vector<float>> A02 = Trans(0, PI/2, 0, -PI/2);
			std::vector<std::vector<float>> A1 = matrixMulti(A01,A02);
			std::vector<std::vector<float>> A2 = Trans(legLength, 0, 0, t2);
			std::vector<std::vector<float>> A3 = Trans(calfLength, 0, 0, t3);
			A1 = matrixMulti(P04[i],A1);

			std::vector<std::vector<float>> T2 = matrixMulti(A1, A2);
			std::vector<std::vector<float>> T3 = matrixMulti(T2, A3);


			//creating zi
			std::vector<float> z0 = {P04[i][0][2], P04[i][1][2], P04[i][2][2]};
			std::vector<float> z1 = {A1[0][2], A1[1][2], A1[2][2]};
			std::vector<float> z2 = {T2[0][2], T2[1][2], T2[2][2]};
			std::vector<float> z3 = {T3[0][2], T3[1][2], T3[2][2]};

			

			//creating pi
			std::vector<float> p0 = {P04[i][0][3], P04[i][1][3], P04[i][2][3]};
			std::vector<float> p1 = {A1[0][3], A1[1][3], A1[2][3]};
			std::vector<float> p2 = {T2[0][3], T2[1][3], T2[2][3]};
			std::vector<float> P = {T3[0][3], T3[1][3], T3[2][3]};

			//Jacobian matrix
			std::vector<std::vector<float>> J(6, std::vector<float>(3));
			std::vector<float> aux1 = cross(z0, {P[0]-p0[0],P[1]-p0[1],P[2]-p0[2]});
			std::vector<float> aux2 = cross(z1, {P[0]-p1[0],P[1]-p1[1],P[2]-p1[2]});
			std::vector<float> aux3 = cross(z2, {P[0]-p2[0],P[1]-p2[1],P[2]-p2[2]});
			J[0] = {aux1[0], aux2[0], aux3[0]};
			J[1] = {aux1[1], aux2[1], aux3[1]};
			J[2] = {aux1[2], aux2[2], aux3[2]};
			J[3] = {z0[0], z1[0], z2[0]};
			J[4] = {z0[1], z1[1], z2[1]};
			J[5] = {z0[2], z1[2], z2[2]};

		}

	}
}




int main(int argc, char** argv){

	// ros::init(argc, argv, "controllerTesting");
	// ros::NodeHandle n;


	// //waiting ros node got ready
	// ros::Time last_ros_time_;
	// bool wait = true;
	// while(wait){
	// 	last_ros_time_ = ros::Time::now();
	// 	if (last_ros_time_.toSec() > 0)
	// 		wait = false;
	// }

	// //ros::Time time = ros::Time::now();

	// std::vector<ros::Publisher> jointPub(nJoints);

	// jointState.name.resize(nJoints);
	// jointState.position.resize(nJoints);

	// std::sort(jointName.begin(), jointName.end());

	// for(int i = 0; i < nJoints; i++)
	// 	jointPub[i] = n.advertise<std_msgs::Float64>("/quadrupedal_test/" + jointName[i] + "_controller/command", 5);
	
	// ros::Subscriber sub = n.subscribe("/quadrupedal_test/joint_states", 10, jointCallback);
	// ros::Rate r(RATE);
	
	
	// std::string robot_desc_string;
	// n.param("robot_description", robot_desc_string, std::string());


	// sensor_msgs::JointState crouchState = crouch(jointName);

	//while(ros::ok()){

		

		// for(int i = 0; i < nJoints; i++){			
			

		// 	std_msgs::Float64 msg;
		// 	msg.data = crouchState.position[i];
		// 	//msg.data = 1.5;
		// 	jointPub[i].publish(msg);
		// }



		//ros::spinOnce();
	//}

}