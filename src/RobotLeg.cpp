#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf/model.h"


const double bodyLength = 4;
const double bodyWidth = 1.5;
const double shoulderLength = .05;
const double shoulderRadius = .1;
const double legLength = 2;
const double legRadius = .1;
const double elbowRadius = .1;
const double footRadius = .1;
const int nJoints = 16;
const double PI = 3.14159;

enum leg_type {left_front, right_front, left_back, right_back};

class RobotLeg{
	public:
		RobotLeg(const std::string name);
		RobotLeg(const std::string name, bool up_down, std::string urdfParam, double timeOut, double eps);
		void init();
		void setUpDown(const bool up_down);
		void setEndPosition(const std::vector<double>& pose);
		bool getJointArray(std::vector<double>& joints);
		bool UpDown(){
			return up_down;
		}

	private:
		std::string name;
		leg_type leg;
		bool up_down;
		std::string urdfParam;
		double timeOut;
		double eps;

		//shoulder to foot
		TRAC_IK::TRAC_IK* tracSolver;
		KDL::ChainFkSolverPos_recursive* fkSolver;
		KDL::Chain chain;
		KDL::JntArray jointArray;
		KDL::JntArray lowerB, higherB;
		std::string chainStart;
		std::string chainEnd;
		std::vector<double> Beg2End;

		//foot to shoulder
		TRAC_IK::TRAC_IK* inv_tracSolver;
		KDL::ChainFkSolverPos_recursive* inv_fkSolver;
		KDL::Chain inv_chain;
		KDL::JntArray inv_jointArray;
		KDL::JntArray inv_lowerB, inv_higherB;
		std::string inv_chainStart;
		std::string inv_chainEnd;
		std::vector<double> inv_Beg2End;

		KDL::Frame targetFrame;

};

void RobotLeg::init(){

	if(name == "left_front")
		leg = left_front;
	else if(name == "right_front")
		leg = right_front;
	else if(name == "left_back")
		leg = left_back;
	else if(name == "right_back")
		leg = right_back;

	chainStart = "base_link";
	chainEnd = name + "_foot";

	tracSolver = new TRAC_IK::TRAC_IK(chainStart, chainEnd, urdfParam, timeOut, eps);
	bool valid = tracSolver->getKDLChain(chain);
	if(!valid)
		ROS_ERROR("No valid chain");

	valid = tracSolver->getKDLLimits(lowerB, higherB);
	if(!valid)
		ROS_ERROR("No valid limits");

	fkSolver = new KDL::ChainFkSolverPos_recursive(chain);
	jointArray = KDL::JntArray(chain.getNrOfJoints());
	std::vector<std::vector<double>> body2foot {{1.5, .875, -4.1, 0, 0, 0},
												{1.5, -.875, -4.1, 0, 0, 0},
												{-1.5, .875, -4.1, 0, 0, 0},
												{-1.5, -.875, -4.1, 0, 0, 0}};
	jointArray(0) = 0;
	jointArray(1) = 0;
	jointArray(2) =	0;
	jointArray(3) =	0;
	switch(leg){
		case left_front: Beg2End = body2foot[0]; break;
		case right_front: Beg2End = body2foot[1]; break;
		case left_back: Beg2End = body2foot[2]; break;
		case right_back: Beg2End = body2foot[3]; break;
	}

	inv_chainStart = name + "_foot";
	inv_chainEnd = "base_link";

	inv_tracSolver = new TRAC_IK::TRAC_IK(inv_chainStart, inv_chainEnd, urdfParam, timeOut, eps);
	valid = inv_tracSolver->getKDLChain(inv_chain);
	if(!valid)
		ROS_ERROR("No valid chain");

	valid = tracSolver->getKDLLimits(inv_lowerB, inv_higherB);
	if(!valid)
		ROS_ERROR("No valid limits");

	inv_fkSolver = new KDL::ChainFkSolverPos_recursive(chain);
	inv_jointArray = KDL::JntArray(chain.getNrOfJoints());
	std::vector<std::vector<double>> foot2body {{-1.5, -.875, 4.1, 0, 0, 0},
												{-1.5, .875, 4.1, 0, 0, 0},
												{1.5, -.875, 4.1, 0, 0, 0},
												{1.5, .875, 4.1, 0, 0, 0}};

	inv_jointArray(0) = 0;
	inv_jointArray(1) = 0;
	inv_jointArray(2) =	0;
	inv_jointArray(3) =	0;
	switch(leg){
		case left_front: inv_Beg2End = foot2body[0]; break;
		case right_front: inv_Beg2End = foot2body[1]; break;
		case left_back: inv_Beg2End = foot2body[2]; break;
		case right_back: inv_Beg2End = foot2body[3]; break;
	}
}

RobotLeg::RobotLeg(const std::string name):
	name(name),
	up_down(true),
	urdfParam("/robot_description"),
	timeOut(0.005),
	eps(2e-5)	
{
	init();
}

RobotLeg::RobotLeg(const std::string name, bool up_down, std::string urdfParam, double timeOut, double eps):
	name(name),
	up_down(up_down),
	urdfParam(urdfParam),
	timeOut(timeOut),
	eps(eps)
{
	init();
}

void RobotLeg::setUpDown(const bool up_down_){
	up_down = up_down_;
}

void RobotLeg::setEndPosition(const std::vector<double>& position){
	std::vector<double> bias;
	if(up_down)
		bias = Beg2End;
	else
		bias = inv_Beg2End;

	double x = position[0] + bias[0];
	double y = position[1] + bias[1];
	double z = position[2] + bias[2];
	double R = position[3] + bias[3];
	double P = position[4] + bias[4];
	double Y = position[5] + bias[5];
	KDL::Vector v(x, y, z);
	targetFrame = KDL::Frame(KDL::Rotation::RPY(R,P,Y), v);	
}

bool RobotLeg::getJointArray(std::vector<double>& joints){
	KDL::JntArray result;
	int rc = 0;
	if(up_down){
		rc = tracSolver->CartToJnt(jointArray, targetFrame, result);
		for(size_t i = 0, j= chain.getNrOfJoints()-1; i < chain.getNrOfJoints(); i++, j--)
			inv_jointArray(j) = jointArray(i);
	}else{
		rc = inv_tracSolver->CartToJnt(inv_jointArray, targetFrame, result);
		for(size_t i = 0, j= chain.getNrOfJoints()-1; i < chain.getNrOfJoints(); i++, j--)
			jointArray(j) = inv_jointArray(i);
	}
	if(rc < 0)
		return false;
	else{
		joints.clear();
		if(up_down)
			for(size_t i = 0; i < chain.getNrOfJoints(); i++)
				joints.push_back(result(i));
		else
			for(int i = chain.getNrOfJoints()-1; i > -1; i--)
				joints.push_back(result(i));
	return true;
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "RobotLeg");
	ros::NodeHandle n;
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	//ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("joint_states",1);
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(30);

	std::string robot_desc_string;
	n.param("robot_description", robot_desc_string, std::string());

	std::vector<std::string> jointName= {"left_front_leg_shoulder_rotate", "right_front_leg_shoulder_rotate", "left_back_leg_shoulder_rotate", "right_back_leg_shoulder_rotate",
											"left_front_leg_shoulder_hinge", "right_front_leg_shoulder_hinge", "left_back_leg_shoulder_hinge", "right_back_leg_shoulder_hinge",
											"left_front_leg_elbow_hinge", "right_front_leg_elbow_hinge", "left_back_leg_elbow_hinge", "right_back_leg_elbow_hinge",
											"left_front_calf_elbow_hinge", "right_front_calf_elbow_hinge", "left_back_calf_elbow_hinge", "right_back_calf_elbow_hinge"};

	// for joints publication
	sensor_msgs::JointState jointState;
	// for odom publication
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "dummy";
	odom_trans.child_frame_id = "base_link";

	RobotLeg left_front_leg("left_front");
	RobotLeg right_front_leg("right_front");
	RobotLeg left_back_leg("left_back");
	RobotLeg right_back_leg("right_back");
	std::vector<RobotLeg> legs{left_front_leg, right_front_leg, left_back_leg, right_back_leg};

	std::vector<std::vector<double>> endPos(4, std::vector<double>(6, 0.0));
	std::vector<std::vector<double>> result(4, std::vector<double>(4, 0.0));

	int flag =-1;
	double x = 0;
	double y = 0;
	double z = 0;
	double yaw = 0;
	double pitch = 0;
	while(ros::ok()){

		legs[0].setUpDown(false);
		legs[1].setUpDown(false);
		legs[2].setUpDown(false);
		legs[3].setUpDown(false);

		if(x > 2 || x < -2)
			flag = -flag;

		x += .5 * flag;
		y += .005 * flag;
		z += .002 * flag;

		yaw = .007 * flag;
		pitch = .003 * flag;

		for(int i = 0; i < 4; i++){
			legs[i].setEndPosition(endPos[i]);
			if(!legs[i].getJointArray(result[i])){
				std::cout << "not success" << std::endl;
				return -1;
			}else
				std::cout << "success" << std::endl;
		}
		ROS_INFO("update joint state");

		jointState.header.stamp = ros::Time::now();
		jointState.name.resize(nJoints);
		jointState.position.resize(nJoints);
		std::vector<double> newPos;

		for(int i = 0; i < 4; i++)
			for(int j = 0; j < 4; j++)
				newPos.push_back(result[i][j]);
		for(size_t i = 0; i < nJoints; i++){
			jointState.name[i] = jointName[i];
			jointState.position[i] = newPos[i];
		}

		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 4.1 + z;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(pitch, yaw, 0);
		ROS_INFO("pub joint state");
		joint_pub.publish(jointState);
		ROS_INFO("pub dom state");
		broadcaster.sendTransform(odom_trans);

		loop_rate.sleep();

	}
}