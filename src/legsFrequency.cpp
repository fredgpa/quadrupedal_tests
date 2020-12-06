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
#include "std_msgs/String.h"
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

#define PI 3.14159
#define RATE 100
#define F_ROT 2*PI

static const int nLegs = 4;

std::string flag;
ros::Time start;

bool TrbMoving;
bool TrfMoving;
bool TlfMoving;
bool TlbMoving;


ros::Time Trb;
ros::Time Trf;
ros::Time Tlf;
ros::Time Tlb;

bool once = true;

void flagCallback(const std_msgs::String::ConstPtr& msg){

	if(once){
		start = ros::Time::now();
		once = !once;
	}
	flag = msg->data.c_str();

	Trb = ros::Time::now();
	TrbMoving = true;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "legsFrequency");
	ros::NodeHandle n;

	ros::Time last_ros_time_;
	bool wait = true;
	while(wait){
		last_ros_time_ = ros::Time::now();
		if (last_ros_time_.toSec() > 0)
			wait = false;
	}

	ros::Rate r(RATE);


	std::vector<ros::Publisher> legPub(nLegs);

	std::vector<std::string> legs = {"RB", "RF", "LF", "LB"};

	for(int i = 0; i < nLegs; i++)
		legPub[i] = n.advertise<std_msgs::String>("/quadrupedal_test/legState"+legs[i], 1000);

	ros::Subscriber flagSub = n.subscribe("/quadrupedal_test/flag", 1000, flagCallback);

	std_msgs::String msg;
	std::stringstream ss;

	bool aux1 = true;
	bool aux2 = true;
	bool aux3 = true;

	while(ros::ok()){

		if(flag == "start"){

			std::cout << ros::Time::now().toSec() << std::endl;
			std::cout << Trb.toSec() << std::endl;
			if(Trb.toSec() - ros::Time::now().toSec() >= 2*.167 && aux1){
				Tlf = ros::Time::now();
				TlfMoving = true;
			}else if(Trb.toSec() - ros::Time::now().toSec() >= 4*.167 && aux2){
				Trf = ros::Time::now();
				TrfMoving = true;
			}else if(Trb.toSec() - ros::Time::now().toSec() >= 6*.167 && aux3){
				Tlb = ros::Time::now();
				TlbMoving = true;
			}


			//Right Back Leg
			if(TrbMoving){
				if(Trb.toSec() - ros::Time::now().toSec() < .5){

					ss << "move";
					msg.data = ss.str();
					legPub[0].publish(msg);
				}else{
					Trb = ros::Time::now();
					TrbMoving = false;
				}
			}else{
				if(Trb.toSec() - ros::Time::now().toSec() < 5*.167){

					ss << "stop";
					msg.data = ss.str();
					legPub[0].publish(msg);
				}else{
					Trb = ros::Time::now();
					TrbMoving = true;
				}
			}

			//Right Front Leg
			if(TrfMoving){
				if(Trf.toSec() - ros::Time::now().toSec() < .5){

					ss << "move";
					msg.data = ss.str();
					legPub[0].publish(msg);
				}else{
					Trf = ros::Time::now();
					TrfMoving = false;
				}
			}else{
				if(Trf.toSec() - ros::Time::now().toSec() < 5*.167){

					ss << "stop";
					msg.data = ss.str();
					legPub[0].publish(msg);
				}else{
					Trf = ros::Time::now();
					TrfMoving = true;
				}
			}

			//Left Front Leg
			if(TlfMoving){
				if(Tlf.toSec() - ros::Time::now().toSec() < .5){

					ss << "move";
					msg.data = ss.str();
					legPub[0].publish(msg);
				}else{
					Tlf = ros::Time::now();
					TlfMoving = false;
				}
			}else{
				if(Tlf.toSec() - ros::Time::now().toSec() < 5*.167){

					ss << "stop";
					msg.data = ss.str();
					legPub[0].publish(msg);
				}else{
					Tlf = ros::Time::now();
					TlfMoving = true;
				}
			}


			//Left Back Leg
			if(TlbMoving){
				if(Tlb.toSec() - ros::Time::now().toSec() < .5){

					ss << "move";
					msg.data = ss.str();
					legPub[0].publish(msg);
				}else{
					Tlb = ros::Time::now();
					TlbMoving = false;
				}
			}else{
				if(Tlb.toSec() - ros::Time::now().toSec() < 5*.167){

					ss << "stop";
					msg.data = ss.str();
					legPub[0].publish(msg);
				}else{
					Tlb = ros::Time::now();
					TlbMoving = true;
				}
			}

		}

		ros::spinOnce();
		r.sleep();
	}
}