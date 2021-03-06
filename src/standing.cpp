#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "urdf/model.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "standing_test_node");
	ros::NodeHandle n;
	ros::Rate r(100);
	//ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	tf::TransformBroadcaster broadcaster;

	std::string robot_desc_string = "model";
	n.param("robot_description", robot_desc_string, std::string());

	std::vector<std::string> joint_name= {"left_front_leg_shoulder_rotate", "right_front_leg_shoulder_rotate", "left_back_leg_shoulder_rotate", "right_back_leg_shoulder_rotate",
											"left_front_leg_shoulder_hinge", "right_front_leg_shoulder_hinge", "left_back_leg_shoulder_hinge", "right_back_leg_shoulder_hinge",
											"left_front_leg_elbow_hinge", "right_front_leg_elbow_hinge", "left_back_leg_elbow_hinge", "right_back_leg_elbow_hinge",
											"left_front_calf_elbow_hinge", "right_front_calf_elbow_hinge", "left_back_calf_elbow_hinge", "right_back_calf_elbow_hinge"};


	//tf::Transform transform;
	//transform.setOrigin()


	while(n.ok()){
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.5, .875, -4.1)), ros::Time::now(),"base_link", "left_front_foot"));

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.5, -.875, -4.1)), ros::Time::now(),"base_link", "right_front_foot"));

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1.5, .875, -4.1)), ros::Time::now(),"base_link", "left_back_foot"));

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1.5, -.875, -4.1)), ros::Time::now(),"base_link", "right_back_foot"));
		r.sleep();
	}
}