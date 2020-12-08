#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <armadillo>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

//=========================== VARIABLES =======================================\\

//--------------------------- Constants ---------------------------------------\\

#define PI 3.14159
#define RATE 1000
#define F_ROT 2*PI

//--------------------------- ROBOT PARAMETERS --------------------------------\\
// currently for model2.urdf

static const int nJoints = 12;
static const int nLegs = 4;

static const float bodyLength = 1;
static const float bodyWidth = .4;
static const float shoulderLength = .1;
static const float legLength = .4;
static const float calfLength = .4;
static const float weight = 100;

static const float omega = 0;
static const float phi = 0;
static const float psi = 0;
static const float xm = 0;
static const float ym = 0;
static const float zm = .8;
static const std::vector<float> cm = {xm, ym, zm};

std::vector<double> lastStep(nLegs, 0.0);


//--------------------------- Joint Names -------------------------------------\\

std::vector<std::string> jointName= {"left_front_leg_shoulder_rotate", "right_front_leg_shoulder_rotate", "left_back_leg_shoulder_rotate", "right_back_leg_shoulder_rotate",
											"left_front_leg_shoulder_hinge", "right_front_leg_shoulder_hinge", "left_back_leg_shoulder_hinge", "right_back_leg_shoulder_hinge",
											"left_front_leg_elbow_hinge", "right_front_leg_elbow_hinge", "left_back_leg_elbow_hinge", "right_back_leg_elbow_hinge"};



//--------------------------- Variable to receive robot info ------------------\\

sensor_msgs::JointState jointState;
std::string TrbMoving;
std::string TrfMoving;
std::string TlfMoving;
std::string TlbMoving;

//=============================================================================\\
//=========================== FUNCTIONS =======================================\\
//=============================================================================\\

// -------------------------- ROS Stuff ---------------------------------------\\

void jointCallback(const sensor_msgs::JointState::ConstPtr &_js){
//translates ROS joint_states message to the global variable

	jointState.header.seq = _js->header.seq;
	jointState.header.stamp = _js->header.stamp;
	jointState.header.frame_id = _js->header.frame_id;

	for(size_t i = 0; i < nJoints; i++){
		//
		jointState.name[i] = _js->name[i];		
		jointState.position[i] = _js->position[i];
	}
}

void legCallbackTrb(const std_msgs::String::ConstPtr& msg){

	TrbMoving = msg->data.c_str();

}

void legCallbackTrf(const std_msgs::String::ConstPtr& msg){

	TrfMoving = msg->data.c_str();

}

void legCallbackTlf(const std_msgs::String::ConstPtr& msg){

	TlfMoving = msg->data.c_str();

}

void legCallbackTlb(const std_msgs::String::ConstPtr& msg){

	TlbMoving = msg->data.c_str();

}

void printJointState(sensor_msgs::JointState jointState_){
//prints data on joint state variable

	for(int i = 0; i < nJoints; i++){
		std::cout << "Joint Name: " <<std::endl;
		std::cout << jointState_.name[i] <<std::endl;
		std::cout << "Joint Position: " <<std::endl;
		std::cout << jointState_.position[i] <<std::endl;
	}
}

// -------------------------- Movement Templates ------------------------------\\

sensor_msgs::JointState crouch(std::vector<std::string> jointName){
//radians for each joint to get the robot on a crouching stance
	sensor_msgs::JointState jointState_;

	jointState_.name.resize(nJoints);
	jointState_.position.resize(nJoints);

	jointState_.name = jointName;

	for(int i = 0; i < nJoints; i++){
		if(jointState_.name[i].find("_shoulder_hinge") != std::string::npos){

			jointState_.position[i] = 0;

		}else if(jointState_.name[i].find("_shoulder_rotate") != std::string::npos){
			if(jointState_.name[i].find("right_back") != std::string::npos || jointState_.name[i].find("left_front") != std::string::npos)
				jointState_.position[i] = -.28;
			else
				jointState_.position[i] = .28;			
		}else if(jointState_.name[i].find("_leg_elbow") != std::string::npos){
			if(jointState_.name[i].find("right_back") != std::string::npos || jointState_.name[i].find("left_front") != std::string::npos)
				jointState_.position[i] = .6;
			else
				jointState_.position[i] = -.6;
		}
	}

	return jointState_;
}

sensor_msgs::JointState standUp(std::vector<std::string> jointName_){
//radians for each joint to get the robot on a stood up stance
	sensor_msgs::JointState jointState_;

	jointState_.name.resize(nJoints);
	jointState_.position.resize(nJoints);

	jointState_.name = jointName_;

	for(int i = 0; i < nJoints; i++){
		jointState_.position[i] = 0;
	}

	return jointState_;
}

sensor_msgs::JointState showOff(sensor_msgs::JointState jointState_, bool aux){
//radians for each joint to get the robot to lift some of the legs

	for(int i = 0; i < nJoints; i++){
		if(jointState_.name[i].find("_shoulder_rotate") != std::string::npos){
			if(aux){
				if(jointState_.name[i].find("right_back") != std::string::npos)
					jointState_.position[i] = .9;
				else if(jointState_.name[i].find("left_back") != std::string::npos)
					jointState_.position[i] = -.28;
			}else{
				if(jointState_.name[i].find("right_back") != std::string::npos)
					jointState_.position[i] = .28;
				else if(jointState_.name[i].find("left_back") != std::string::npos)
					jointState_.position[i] = -.9;
			}
		}else if(jointState_.name[i].find("_leg_elbow") != std::string::npos){
			if(aux){
				if(jointState_.name[i].find("right_back") != std::string::npos)
					jointState_.position[i] = -1.57;
				else if(jointState_.name[i].find("left_back") != std::string::npos)
					jointState_.position[i] = .6;
			}else{
				if(jointState_.name[i].find("right_back") != std::string::npos)
					jointState_.position[i] = -.6;
				else if(jointState_.name[i].find("left_back") != std::string::npos)
					jointState_.position[i] = 1.57;
			}
		}
	}

	return jointState_;
}


std::vector<std::vector<float>> sinWalking(double instant, std::vector<std::vector<float>> currentPoint){
//returns the foot position in function of time variable
	std::vector<std::vector<float>> point(nLegs);
	//2.1 segundos uma passada completa
	// std::vector<float> legTiming = {4*.167, 0, 2*.167, 6*.167};
	std::vector<float> legTiming = { 2*PI/15, 0, PI/5, PI/15};

	float x;
	float y;
	float z;

	std::vector<bool> legFirst = {true, true, true, true};

	std::vector<float> instantLegs = {instant-legTiming[0], instant-legTiming[1], instant-legTiming[2], instant-legTiming[3]};

	// std::cout << instantLegs[1] << std::endl;

	for(int i = 0; i < nLegs; i++){
		// std::cout << instant << std::endl;

		x = currentPoint[i][0];//calculate z
		y = currentPoint[i][1];
		z = currentPoint[i][2];

		if(instant >= legTiming[i]){

			if(lastStep[i] == 0){
				
				z = sin((instantLegs[i])*10)*.6;
					
				if((instantLegs[i]) >= PI/8)
					lastStep[i] = (instantLegs[i]);
			}else if((instantLegs[i]) >= lastStep[i] + PI/6){

				z = (sin(instantLegs[i] - lastStep[i] - PI/6)*10)*.6;

				if((instantLegs[i]) >= lastStep[i] + PI/5)
					lastStep[i] = (instantLegs[i]);

			}else
				z = 0;

			// std::cout << z << std::endl;
			// std::cout << "-------------------------------" << std::endl;

			if(z <= 0)
				z = 0;
			else{
				if(z > .2)
					z = .2;

					// if( i == 1 || i == 2)
						x += .0009;
					// else
						// x -= .02;
					// else
				 // 		x -= .1;

			}
		}


		point[i] = {x, y, z};
	}

	return point;

}


//=========================== Mathematical functions ==========================\\


//--------------------------- Miscellaneous -----------------------------------\\

std::vector<float> cross(std::vector<float> v1, std::vector<float> v2){
//creates a vector third perpendicular vector
	std::vector<std::vector<float>> matrix(3,std::vector<float>(3));
	std::vector<float> vector(3);
	matrix[0] = {1, 1, 1};
	matrix[1] = v1;
	matrix[2] = v2;

	vector = {(matrix[1][1]*matrix[2][2] - matrix[1][2]*matrix[2][1]), (matrix[1][2]*matrix[2][0] - matrix[1][0]*matrix[2][2]), (matrix[1][0]*matrix[2][1] - matrix[1][1]*matrix[2][0])};

	return vector;
}

float dot(std::vector<float> v1, std::vector<float> v2){
//calculates the scalar product of two vectors
	float value = 0;

	if (v1.size() == v2.size())
		for(int i = 0; i < v1.size(); i++)
			value += v1[i]*v2[i];
	else
		ROS_ERROR("The vectors have different dimmensions!");

	return value;
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

//--------------------------- Theta handling ----------------------------------\\

std::vector<float> calculateTheta(std::string type, sensor_msgs::JointState jointState_){
//search in a joint state variable for radian values on a specific type of joint
	std::vector<float> theta(nLegs);

	for(int i = 0; i < nJoints; i++){
		if(jointState_.name[i].find(type) != std::string::npos){
			if(jointState_.name[i].find("right_back") != std::string::npos)

				theta[0] = jointState_.position[i];

			else if(jointState_.name[i].find("right_front") != std::string::npos)

				theta[1] = jointState_.position[i];

			else if(jointState_.name[i].find("left_front") != std::string::npos)

				theta[2] = jointState_.position[i];

			else

				theta[3] = jointState_.position[i];
		}
	}


	return theta;
}

sensor_msgs::JointState insertTheta(std::vector<float> theta, sensor_msgs::JointState jointState_, std::string leg){
//inserts radian values on specific joints of a joint state variable
	for(int i = 0; i < jointState_.name.size(); i++){
		if(jointState_.name[i].find(leg) != std::string::npos){
			if(jointState_.name[i].find("shoulder_hinge") != std::string::npos)
				jointState_.position[i] = theta[0];
			else if(jointState_.name[i].find("shoulder_rotate") != std::string::npos)
				jointState_.position[i] = theta[1];
			else if(jointState_.name[i].find("elbow_hinge") != std::string::npos)
				jointState_.position[i] = theta[2];
		}

			
	}

	return jointState_;
}

//--------------------------- Matrix-related ----------------------------------\\

std::vector<std::vector<float>> matrixMulti(std::vector<std::vector<float>> m1, std::vector<std::vector<float>> m2){
//multiplies two matrixes
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

std::vector<float> matrixArrayMulti(std::vector<std::vector<float>> matrix, std::vector<float> array){
//multiply a matrix and an array
	std::vector<float> multi(matrix.size());

		for(int i = 0; i < matrix.size(); i++){
			multi[i] = 0;
			for(int j = 0; j < matrix[0].size(); j++)
				multi[i] += matrix[i][j]*array[j];
		}


 	return multi;
}

std::vector<std::vector<float>> matrixSumSub(std::vector<std::vector<float>> m1, std::vector<std::vector<float>> m2, bool aux){
//sums or substracts two matrixes
	std::vector<std::vector<float>> matrix(m1.size(), std::vector<float>(m1[0].size()));

	for(int i = 0; i < m1.size(); i++)
 		for(int j = 0; j < m1[0].size(); j++){
 			if (aux)
 				matrix[i][j] = m1[i][j] + m2[i][j];
 			else
 				matrix[i][j] = m1[i][j] - m2[i][j];
 		}

	return matrix;
}

std::vector<std::vector<float>> matrixNumberMultiDiv(std::vector<std::vector<float>> matrix, float number, bool aux){
//multiplies or divide a matrix by a number
	std::vector<std::vector<float>> result(matrix.size(), std::vector<float>(matrix[0].size()));

	for(int i = 0; i < matrix.size(); i++)
		for(int j = 0; j < matrix[0].size(); j++){
			if(aux)
				result[i][j] = matrix[i][j] * number;
			else if(number != 0)
				result[i][j] = matrix[i][j] / number;
		}

	return result;
}

std::vector<std::vector<float>> matrixTransp(std::vector<std::vector<float>> a){
//finds the transposed matrix
	std::vector<std::vector<float>> matrix(a.size(), std::vector<float>(a[0].size()));

	for (int i = 0; i < a[0].size(); i++)
		for (int j = 0; j < a.size(); j++)
			matrix[j][i] = a[i][j];

	return matrix;
}

std::vector<std::vector<float>> createIdent(int n){
//create a matrix of zeros with ones at the main diagonal
	std::vector<std::vector<float>> matrix(n, std::vector<float>(n, 0.0));

	for(int i = 0; i < n; i++)
		for(int j = 0; j < n; j++)
			if (i == j)
				matrix[i][j] = 1;

	return matrix;
}

std::vector<std::vector<float>> invertMatrix(std::vector<std::vector<float>> matrix){
//pseudo-inverts a matrix
	arma::fmat inv(3,3);

	inv(0,0) = matrix[0][0];
	inv(0,1) = matrix[0][1];
	inv(0,2) = matrix[0][2];
	inv(1,0) = matrix[1][0];
	inv(1,1) = matrix[1][1];
	inv(1,2) = matrix[1][2];
	inv(2,0) = matrix[2][0];
	inv(2,1) = matrix[2][1];
	inv(2,2) = matrix[2][2];

	inv = arma::pinv(inv);

	matrix[0][0] = inv(0,0);
	matrix[0][1] = inv(0,1);
	matrix[0][2] = inv(0,2);
	matrix[1][0] = inv(1,0);
	matrix[1][1] = inv(1,1);
	matrix[1][2] = inv(1,2);
	matrix[2][0] = inv(2,0);
	matrix[2][1] = inv(2,1);
	matrix[2][2] = inv(2,2);

	return matrix;
}

std::vector<std::vector<float>> simplerMatrix(std::vector<std::vector<std::vector<float>>> a){
//simplifies a vector<vector<vector into a vector<vector
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

std::vector<std::vector<float>> elementWiseMatrixMultiplication(std::vector<std::vector<float>> a, std::vector<std::vector<float>> b){
//multiply every matrix element by its correspondant on the other (a[i][j]*b[i][j])
	std::vector<std::vector<float>> array(a.size(), std::vector<float>(a[0].size()));

	for(int i = 0; i < a.size(); i++)
		for(int j = 0; j < a[0].size(); j++)
			array[i][j] = a[i][j] * b[i][j];

	return array;
}

float sumMatrix(std::vector<std::vector<float>> array){
//sums all the values of a matrix
	float result = 0;
	for(int i = 0; i < array.size(); i++)
		for(int j = 0; j < array[0].size(); j++)
			result += array[i][j];
	

	return result;
}

//--------------------------- Vector-related ----------------------------------\\

std::vector<float> vectorNumberMultiDiv(std::vector<float> vector, float number, bool aux){
//divides or multiply a vector by a number
	std::vector<float> result(vector.size());

	for(int i = 0; i < vector.size(); i++){
			if(aux)
				result[i] = vector[i] * number;
			else if(number != 0)
				result[i] = vector[i] / number;
		}

	return result;
}

std::vector<float> vectorSumSub(std::vector<float> v1, std::vector<float> v2, bool aux){
//sums or substracts two vectors
	std::vector<float> vector(v1.size());

	for(int i = 0; i < v1.size(); i++){
		if (aux)
			vector[i] = v1[i] + v2[i];
		else
			vector[i] = v1[i] - v2[i];
	}

	return vector;
}


float sum(std::vector<float> array){
//sums all the values of a vector
	float result = 0;
	for (int i = 0; i < array.size(); i++){
		result += array[i];
	}

	return result;
}

std::vector<float> elementWiseMultiplication(std::vector<float> a, std::vector<float> b){
//multiply every vector element by its correspondant on the other (a[i]*b[i])
	std::vector<float> array(a.size());

	for(int i = 0; i < a.size(); i++)
		array[i] = a[i] * b[i];

	return array;
}

//--------------------------- Quaternions -------------------------------------\\

std::vector<float> createQuaternion(std::vector<float> vector){
//create a quaternion array
	float a = vector[0];
	float alf = vector[1];
	float d = vector[2];
	float theta = vector[3];

	float Am = cos(alf/2)*cos(theta/2);
	float Bm = sin(alf/2)*cos(theta/2);
	float Cm = sin(alf/2)*sin(theta/2);
	float Dm = cos(alf/2)*sin(theta/2);	

	std::vector<float> quaternion = {Am,
									 Bm, 
									 Cm, 
									 Dm, 
									 -(a/2)*Bm-(d/2)*Dm,
									 (a/2)*Am-(d/2)*Cm,
									 (a/2)*Dm+(d/2)*Bm,
									 -(a/2)*Cm+(d/2)*Am};

	return quaternion;
}

std::vector<float> QProduct(std::vector<float> q1, std::vector<float> q2){
//calculate the product of two quaternion vectors
	float q_Part1 = q1[0]*q2[0] - dot(
									{q1[1], q1[2], q1[3]}, 
									{q2[1], q2[2], q2[3]});

	std::vector<float> q_Part2 = vectorSumSub(
									vectorSumSub(
										vectorNumberMultiDiv(
											{q2[1], q2[2], q2[3]}, 
											q1[0], 
											true), 
										vectorNumberMultiDiv(
											{q1[1], q1[2], q1[3]}, 
											q2[0], 
											true), 
										true), 
								 cross(
									{q1[1], q1[2], q1[3]}, 
									{q2[1], q2[2], q2[3]}), 
								 true);

	std::vector<float> q(q_Part2.size() + 1);

	q[0] = q_Part1;
	for(int i = 1; i < q.size(); i++)
		q[i] = q_Part2[i-1];

	
	

	return q;
}

std::vector<float> QDualProduct(std::vector<float> q1, std::vector<float> q2){
//calculate the dual product of two quaternions
	std::vector<float> productPart1(q1.size());
	productPart1 = QProduct(
							{q1[0], q1[1], q1[2], q1[3]}, 
							{q2[0], q2[1], q2[2], q2[3]});
	std::vector<float> productPart2 = vectorSumSub(
										QProduct(
											{q1[0], q1[1], q1[2], q1[3]}, 
											{q2[4], q2[5], q2[6], q2[7]}), 
										QProduct(
											{q1[4], q1[5], q1[6], q1[7]}, 
											{q2[0], q2[1], q2[2], q2[3]}), 
										true);

	std::vector<float> product(productPart1.size() + productPart2.size());

	for(int i = 0; i < productPart1.size(); i++)
		product[i] = productPart1[i];

	for(int i = productPart1.size(); i < product.size(); i++){
		product[i] = productPart2[i - productPart1.size()];
	}

	return product;
}

std::vector<float> QConjugate(std::vector<float> q){
//calculate the conjugate

	std::vector<float> conjugate = {q[0], -q[1], -q[2], -q[3], -q[4], q[5], q[6], q[7]};

	return conjugate;
}

//--------------------------- Robot Cinematic ---------------------------------\\

std::vector<std::vector<float>> forwardCinematic(sensor_msgs::JointState jointState_){
//calculates the point of each foot based on the radians of each joint
	// std::vector<float> theta1 = {0, 0, 0, 0};
	// std::vector<float> theta2 = {0, 0, 0, 0};
	// std::vector<float> theta3 = {0, 0, 0, 0};
	// initial leg angle
	std::vector<float> theta1 = calculateTheta("shoulder_hinge", jointState_);
	std::vector<float> theta2 = calculateTheta("shoulder_rotate", jointState_);
	std::vector<float> theta3 = calculateTheta("elbow_hinge", jointState_);

	
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

 	Rz[0][0] = Rz[1][1] = cos(psi);
 	Rz[0][1] = sin(psi);
 	Rz[1][0] = sin(psi);
 	
 	Rxyz = matrixMulti(matrixMulti(Rx, Ry), Rz);
	 
	//center M transformation
 	std::vector<std::vector<float>> matrixAux(nLegs, std::vector<float>(nLegs, 0.0));
 	matrixAux[0][3] = xm;
 	matrixAux[1][3] = ym;
 	matrixAux[2][3] = zm;
 	matrixAux[3][3] = 0;

	std::vector<std::vector<float>> Tm = matrixSumSub(matrixMulti(Rxyz, createIdent(nLegs)),matrixAux, true);

	//Transformation for each leg

	std::vector<std::vector<float>> right_back_leg_matrix(4, std::vector<float>(4));
		right_back_leg_matrix[0] = {0, 0, 1, -bodyLength/2};
		right_back_leg_matrix[1] = {1, 0, 0, -bodyWidth/2};
		right_back_leg_matrix[2] = {0, 1, 0, 0};
		right_back_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> right_front_leg_matrix(4, std::vector<float>(4));
		right_front_leg_matrix[0] = {0, 0, 1, bodyLength/2};
		right_front_leg_matrix[1] = {1, 0, 0, -bodyWidth/2};
		right_front_leg_matrix[2] = {0, 1, 0, 0};
		right_front_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> left_front_leg_matrix(4, std::vector<float>(4));
		left_front_leg_matrix[0] = {0, 0, -1, bodyLength/2};
		left_front_leg_matrix[1] = {-1, 0, 0, bodyWidth/2};
		left_front_leg_matrix[2] = {0, 1, 0, 0};
		left_front_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> left_back_leg_matrix(4, std::vector<float>(4));
		left_back_leg_matrix[0] = {0, 0, -1, -bodyLength/2};
		left_back_leg_matrix[1] = {-1, 0, 0, bodyWidth/2};
		left_back_leg_matrix[2] = {0, 1, 0, 0};
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
	std::vector<std::vector<float>> Torques(4);

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

	
	//std::vector<std::vector<float>> newTorques = simplerMatrix(Torques);	

	// std::cout << "-------------- Posição das Patas ---------------" << std::endl;
	// std::cout << "-------------- Right Back ---------------" << std::endl;
	// std::cout << Patas[0][0] << std::endl;
	// std::cout << Patas[0][1] << std::endl;
	// std::cout << Patas[0][2] << std::endl;
	// std::cout << "-------------- Right Front ---------------" << std::endl;
	// std::cout << Patas[1][0] << std::endl;
	// std::cout << Patas[1][1] << std::endl;
	// std::cout << Patas[1][2] << std::endl;
	// std::cout << "-------------- Left Front ---------------" << std::endl;
	// std::cout << Patas[2][0] << std::endl;
	// std::cout << Patas[2][1] << std::endl;
	// std::cout << Patas[2][2] << std::endl;
	// std::cout << "-------------- Left Back ---------------" << std::endl;
	// std::cout << Patas[3][0] << std::endl;
	// std::cout << Patas[3][1] << std::endl;
	// std::cout << Patas[3][2] << std::endl;

	return Patas;
}

sensor_msgs::JointState inverseCinematic(std::vector<ros::Publisher> jointPub, ros::Rate r, sensor_msgs::JointState jointState_, std::vector<std::vector<float>> objPoint){//sensor_msgs::JointState jointState_){
//calculates the trajectory of each joint to reach the objective point

	// std::vector<float> theta1 = {0, 0, 0, 0};
	// std::vector<float> theta2 = {-PI/3, -PI/3, PI/3, PI/3};
	// std::vector<float> theta3 = {PI/6, PI/6, -PI/6, -PI/6};
	// initial leg angle
	std::vector<float> theta1 = calculateTheta("shoulder_hinge", jointState_);
	std::vector<float> theta2 = calculateTheta("shoulder_rotate", jointState_);
	std::vector<float> theta3 = calculateTheta("elbow_hinge", jointState_);
	

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

 	Rz[0][0] = Rz[1][1] = cos(psi);
 	Rz[0][1] = Rz[1][0] = sin(psi);
 	
 	
 	Rxyz = matrixMulti(matrixMulti(Rx, Ry), Rz);

	 
	//center M transformation
 	std::vector<std::vector<float>> matrixAux(nLegs, std::vector<float>(nLegs, 0.0));
 	matrixAux[0][3] = xm;
 	matrixAux[1][3] = ym;
 	matrixAux[2][3] = zm;
 	matrixAux[3][3] = 0;

	std::vector<std::vector<float>> Tm = matrixSumSub(matrixMulti(Rxyz, createIdent(nLegs)),matrixAux, true);

	//Transformation for each leg

	std::vector<std::vector<float>> right_back_leg_matrix(4, std::vector<float>(4));
		right_back_leg_matrix[0] = {0, 0, 1, -bodyLength/2};
		right_back_leg_matrix[1] = {1, 0, 0, -bodyWidth/2};
		right_back_leg_matrix[2] = {0, 1, 0, 0};
		right_back_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> right_front_leg_matrix(4, std::vector<float>(4));
		right_front_leg_matrix[0] = {0, 0, 1, bodyLength/2};
		right_front_leg_matrix[1] = {1, 0, 0, -bodyWidth/2};
		right_front_leg_matrix[2] = {0, 1, 0, 0};
		right_front_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> left_front_leg_matrix(4, std::vector<float>(4));
		left_front_leg_matrix[0] = {0, 0, -1, bodyLength/2};
		left_front_leg_matrix[1] = {-1, 0, 0, bodyWidth/2};
		left_front_leg_matrix[2] = {0, 1, 0, 0};
		left_front_leg_matrix[3] = {0, 0, 0, 1};
	std::vector<std::vector<float>> left_back_leg_matrix(4, std::vector<float>(4));
		left_back_leg_matrix[0] = {0, 0, -1, -bodyLength/2};
		left_back_leg_matrix[1] = {-1, 0, 0, bodyWidth/2};
		left_back_leg_matrix[2] = {0, 1, 0, 0};
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

	std::vector<std::vector<float>> Theta(4);

	// while
	if(sumMatrix(elementWiseMatrixMultiplication(dx,dx)) >= .005){

		for(int i = 0; i < nLegs; i++){

			float t1 = theta1[i];
			float t2 = theta2[i];
			float t3 = theta3[i];

			std::vector<float> dtheta;

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
			J = {{aux1[0], aux2[0], aux3[0]}, {aux1[1], aux2[1], aux3[1]},
			        {aux1[2], aux2[2], aux3[2]}, {z0[0], z1[0], z2[0]},
			        {z0[1], z1[1], z2[1]}, {z0[2], z1[2], z2[2]}};

			dx[i] = vectorSumSub(objPoint[i], P, false);

			std::vector<std::vector<float>> J_ = {{J[0][0], J[0][1], J[0][2]}, {J[1][0], J[1][1], J[1][2]}, {J[2][0], J[2][1], J[2][2]}};
			J_ = invertMatrix(J_);


			dtheta = matrixArrayMulti(J_, vectorNumberMultiDiv(dx[i],0.01, true));

			Theta[i] = vectorSumSub({t1, t2, t3}, dtheta, true);

			t1 = Theta[i][0];
			t2 = Theta[i][1];
			t3 = Theta[i][2];

		}

		for(int i = 0; i < nLegs; i++){

			for(int j = 0; j < 3; j++){
				if(abs(Theta[i][j]) > F_ROT){
					Theta[i][j] = std::fmod(Theta[i][j], F_ROT);
				}
			}

			if(i == 0)			
				jointState_ = insertTheta(Theta[i], jointState_, "right_back");
			else if(i == 1)
				jointState_ = insertTheta(Theta[i], jointState_, "right_front");
			else if(i == 2)
				jointState_ = insertTheta(Theta[i], jointState_, "left_front");
			else
				jointState_ = insertTheta(Theta[i], jointState_, "left_back");
		}

	}

	// std::cout << "Theta Leg 1: " << std::endl;
	// std:: cout << Theta[0][0] << std::endl;
	// std:: cout << Theta[0][1] << std::endl;
	// std:: cout << Theta[0][2] << std::endl;
	// std::cout << "Theta Leg 2: " << std::endl;
	// std:: cout << Theta[1][0] << std::endl;
	// std:: cout << Theta[1][1] << std::endl;
	// std:: cout << Theta[1][2] << std::endl;
	// std::cout << "Theta Leg 3: " << std::endl;
	// std:: cout << Theta[2][0] << std::endl;
	// std:: cout << Theta[2][1] << std::endl;
	// std:: cout << Theta[2][2] << std::endl;
	// std::cout << "Theta Leg 4: " << std::endl;
	// std:: cout << Theta[3][0] << std::endl;
	// std:: cout << Theta[3][1] << std::endl;
	// std:: cout << Theta[3][2] << std::endl;

	// std::cout << "---------" << std::endl;
	// printJointState(jointState_);
	// std::cout << "---------" << std::endl;



	return jointState_;
}

std::vector<float> forwardCinematicQ(sensor_msgs::JointState jointState_){
//calculates the point of each foot based on the radians of each joint

	std::vector<float> theta1 = {0, 0, 0, 0};
	std::vector<float> theta2 = {-60*PI/180, -60*PI/180, 60*PI/180, 60*PI/180};
	std::vector<float> theta3 = {30*PI/180, 30*PI/180, -30*PI/180, -30*PI/180};
	// initial leg angle
	// std::vector<float> theta1 = calculateTheta("shoulder_hinge", jointState_);
	// std::vector<float> theta2 = calculateTheta("shoulder_rotate", jointState_);
	// std::vector<float> theta3 = calculateTheta("elbow_hinge", jointState_);

	std::vector<std::vector<float>> linha1 = {{-bodyLength/2, PI/2, 0, 0},
											  {bodyLength/2, PI/2, 0, 0},
											  {bodyLength/2, -PI/2, 0, 0},
											  {-bodyLength/2, -PI/2, 0, 0}};

	std::vector<float> linha2 = {0, 0, bodyWidth/2, 0};

	std::vector<std::vector<float>> linha3 = {{0, PI/2, 0, PI/2},
											  {0, PI/2, 0, PI/2},
											  {0, -PI/2, 0, PI/2},
											  {0, -PI/2, 0, PI/2}};

  	std::vector<std::vector<float>> linha4 = {{0, 0, 0, -PI/2},
											  {0, 0, 0, -PI/2},
											  {0, 0, 0, PI/2},
											  {0, 0, 0, PI/2}};

	



	std::vector<float> R_omega = {cos(omega/2), sin(omega/2), 0, 0, 0, 0, 0, 0};
	std::vector<float> R_phi = {cos(phi/2), 0, sin(phi/2), 0, 0, 0, 0, 0};
	std::vector<float> R_psi = {cos(psi/2), 0, 0, sin(psi/2), 0, 0, 0, 0};

	std::vector<std::vector<float>> P01quadril(nLegs);
	std::vector<std::vector<float>> P02quadril(nLegs);
	std::vector<std::vector<float>> Pjoelho(nLegs);
	std::vector<std::vector<float>> Ppata(nLegs);

	for(int i = 0; i < nLegs; i++){

		float t1 = theta1[i];
		float t2 = theta2[i];
		float t3 = theta3[i];
		std::vector<float> p0 = {0, 0, 0, 0, 0, xm, ym, zm};
		std::vector<float> pe = {1, 0, 0, 0, 0, 0, 0, 0};


		std::vector<std::vector<float>> PDH = {{linha1[i][0], linha1[i][1], linha1[i][2], linha1[i][3]},
										   {linha2[0], linha2[1], linha2[2], linha2[3]},
										   {linha3[i][0], linha3[i][1], linha3[i][2], linha3[i][3]},
										   {linha4[i][0], linha4[i][1], linha4[i][2], linha4[i][3]},
										   {-shoulderLength, 0, 0, t1},
										   {0, PI/2, 0, -PI/2},
										   {legLength, 0, 0, t2},
										   {calfLength, 0, 0, t3}};


		std::vector<std::vector<float>> h(PDH.size());
		for(int j = 0; j < PDH.size(); j++)
			h[j] = createQuaternion(PDH[j]);

		std::vector<float> p0e = QDualProduct(R_omega, R_phi);
		p0e = QDualProduct(p0e, R_psi);
		p0e = QDualProduct(p0e, h[0]);

		//quadril
		for(int j = 1; j < 4; j++)
			p0e = QDualProduct(p0e, h[j]);

		std::vector<float> p0eConj = QConjugate(p0e);

		std::vector<float> P1 = QDualProduct(p0e, pe);

		P01quadril[i] = QDualProduct(P1, p0eConj);
		P01quadril[i] = vectorSumSub(P01quadril[i], p0, true);

		//lateral do quadril
		for(int j = 4; j < 6; j++)
			p0e = QDualProduct(p0e, h[j]);

		p0eConj = QConjugate(p0e);
		P1 = QDualProduct(p0e, pe);
		P02quadril[i] = QDualProduct(P1, p0eConj);
		P02quadril[i] = vectorSumSub(P02quadril[i], p0, true);

		//joelho
		p0e = QDualProduct(p0e, h[6]);
		p0eConj = QConjugate(p0e);
		P1 = QDualProduct(p0e, pe);
		Pjoelho[i] = QDualProduct(P1, p0eConj);
		Pjoelho[i] = vectorSumSub(Pjoelho[i], p0, true);

		//pata
		p0e = QDualProduct(p0e, h[7]);
		p0eConj = QConjugate(p0e);
		P1 = QDualProduct(p0e, pe);
		Ppata[i] = QDualProduct(P1, p0eConj);
		Ppata[i] = vectorSumSub(Ppata[i], p0, true);
 
	}



	return Ppata[0];
}




int main(int argc, char** argv){

	//starts the program ROS Node
	ros::init(argc, argv, "controllerTesting");
	ros::NodeHandle n;


	// //waiting ros node to get ready
	ros::Time last_ros_time_;
	bool wait = true;
	while(wait){
		last_ros_time_ = ros::Time::now();
		if (last_ros_time_.toSec() > 0)
			wait = false;
	}

	//creating the publishers that will send our info back to ROS server
	std::vector<ros::Publisher> jointPub(nJoints);
	ros::Publisher legFrequencyStart = n.advertise<std_msgs::String>("/quadrupedal_test/flag", 1000);

	//preparing the global state variable
	jointState.name.resize(nJoints);
	jointState.position.resize(nJoints);

	//arranging the joint names based on rostopic (alphabetically)
	std::sort(jointName.begin(), jointName.end());

	//assign each publisher to a joint
	for(int i = 0; i < nJoints; i++)
		jointPub[i] = n.advertise<std_msgs::Float64>("/quadrupedal_test/" + jointName[i] + "_controller/command", 5);
	
	//create the subscriber and subscribes to the joint_states topic
	ros::Subscriber sub = n.subscribe("/quadrupedal_test/joint_states", 1, jointCallback);
	
	//set a rate to control the loop	
	ros::Rate r(RATE);
	
	//get the robot name from ROSPARAM | not needed right now
	// std::string robot_desc_string;
	// n.param("robot_description", robot_desc_string, std::string());


	//======================= Testing Variables ===============================\\
	//can get very messy

	// sensor_msgs::JointState crouchState = crouch(jointName);
	// std::vector<std::vector<float>> footPos;
	std::vector<std::vector<float>> objPoint;
	std::vector<std::vector<float>> currentPoint;

	
	std::stringstream ss;
	ss << "start";
	std_msgs::String legMsg;
	legMsg.data = ss.str();
	
	//---------------------------------------------------------
	
	bool stance = true;
	sensor_msgs::JointState testeState = crouch(jointName);

	//printJointState(testeState);

	//forwardCinematicQ(testeState);

	//testeState.position[2] = -.9;

	//======================= The Magic Happens Here ==========================\\

	//timing variables
	ros::Time startingLoop = ros::Time::now();



	while(ros::ok()){

		// starts the walking sequence after 5 seconds of loop
		if(ros::Time::now().toSec() - startingLoop.toSec() >= 5){
			// std::cout << ros::Time::now().toSec() - startingLoop.toSec() << std::endl;

			legFrequencyStart.publish(legMsg);

			currentPoint = forwardCinematic(testeState);

			objPoint = sinWalking(ros::Time::now().toSec() - startingLoop.toSec() - 5, currentPoint);

			// std::cout << objPoint[1][0] << std::endl;
			// std::cout << objPoint[1][1] << std::endl;
			// std::cout << objPoint[1][2] << std::endl;
			
			testeState = inverseCinematic(jointPub, r, testeState, objPoint);

			// break;
		}

		


		//starts publishing only after the first joint state message was received
		if(jointState.header.seq != 0){
			for(int i = 0; i < nJoints; i++){
				std_msgs::Float64 msg;
				msg.data = testeState.position[i];
				//msg.data = -.9;
				jointPub[i].publish(msg);
			}
		}

		//controls the subscriber, asks for new messages to be received
		ros::spinOnce();
		//sleeps the necessary time to respect the specified rate value
		r.sleep();

	}

}