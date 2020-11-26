#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <robot_state_publisher/robot_state_publisher.h>

void poseCallback(const turtlesim::Ptr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0));
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argV){
	ros::init(argc, argV, "my_tf_broadcaster");
	if(argc != 2){
		ROS_ERROR("need turtle name as argument");
		return -1;
	}
	turtle_name = argV[1];

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

	ros::spin();
}