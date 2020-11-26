#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include "sensor_msgs/JointState.h"
#include "urdf/model.h"


void transformPoint(const tf::TransformListener& listener){


	geometry_msgs::PointStamped left_front_foot_point;
	left_front_foot_point.header.frame_id = "left_front_foot";
	left_front_foot_point.header.stamp = ros::Time();

	left_front_foot_point.point.x = 5;
	left_front_foot_point.point.y = 4;
	left_front_foot_point.point.z = 0;

	try{
		geometry_msgs::PointStamped base_point;

		listener.transformPoint("base_link", left_front_foot_point, base_point);

		ROS_INFO("left_foot: (%.2f, %.2f, %.2f) ----> base_link: (%.2f, %.2f, %.2f) at time %.2f", 
				left_front_foot_point.point.x, left_front_foot_point.point.y, left_front_foot_point.point.z, 
				base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
	}catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from \"left_front_foot\" to \"base_link\": %s", ex.what());
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_listener");
	ros::NodeHandle n;
	
	ros::Publisher jointPub(n.advertise<sensor_msgs::JointState>("joint_states", 1));
	//ros::Rate r(100); não sei se vou precisar

	std::string robot_desc_string;
	n.param("robot_description", robot_desc_string, std::string());

	std::vector<std::string> joint_name= {"left_front_leg_shoulder_rotate", "right_front_leg_shoulder_rotate", "left_back_leg_shoulder_rotate", "right_back_leg_shoulder_rotate",
											"left_front_leg_shoulder_hinge", "right_front_leg_shoulder_hinge", "left_back_leg_shoulder_hinge", "right_back_leg_shoulder_hinge",
											"left_front_leg_elbow_hinge", "right_front_leg_elbow_hinge", "left_back_leg_elbow_hinge", "right_back_leg_elbow_hinge",
											"left_front_calf_elbow_hinge", "right_front_calf_elbow_hinge", "left_back_calf_elbow_hinge", "right_back_calf_elbow_hinge"};

	//to publish joints
	sensor_msgs::JointState joint_state;



	//tf::TransformListener listener(ros::Duration(10));
	//ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

	ros::spin();
}
