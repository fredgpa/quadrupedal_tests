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


	//tf::Transform transform;
	//transform.setOrigin()

	//Set all the transforms needed
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
