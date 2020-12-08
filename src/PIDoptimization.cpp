#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <armadillo>

#include <dynamic_reconfigure/client.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <effort_controllers/joint_position_controller.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager/controller_loader.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include "ros/service_client.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include "dynamic_reconfigure/ReconfigureRequest.h"
#include "dynamic_reconfigure/Config.h"
#include "dynamic_reconfigure/DoubleParameter.h"

std::vector<std::string> controllerName= {"left_front_leg_shoulder_rotate_controller", "right_front_leg_shoulder_rotate_controller", "left_back_leg_shoulder_rotate_controller", "right_back_leg_shoulder_rotate_controller",
											"left_front_leg_shoulder_hinge_controller", "right_front_leg_shoulder_hinge_controller", "left_back_leg_shoulder_hinge_controller", "right_back_leg_shoulder_hinge_controller",
											"left_front_leg_elbow_hinge_controller", "right_front_leg_elbow_hinge_controller", "left_back_leg_elbow_hinge_controller", "right_back_leg_elbow_hinge_controller"};

const int nControllers = 12;

void commandCallback(){

}

void stateCallback(){
	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "pidTuning");
	ros::NodeHandle n;

	std::vector<ros::ServiceClient> setPID(nControllers);
	std::vector<ros::Subscriber> commandSub(nControllers);
	std::vector<ros::Subscriber> stateSub(nControllers);
	
	for(int i = 0; i < nControllers; i++){
		setPID[i] = n.serviceClient<dynamic_reconfigure::Reconfigure>("/quadrupedal_test/" + controllerName[i] + "/pid/set_parameters");
		commandSub[i] = n.subscribe("/quadrupedal_test/"+ controllerName[i] + "/command", 1, );

	 	stateSub[i] = n.subscribe("/quadrupedal_test/"+ controllerName[i] + "/state", 1, );
	}

	dynamic_reconfigure::Reconfigure srv;

	std::vector<dynamic_reconfigure::DoubleParameter> pid(3);

	pid[0].name = "p";
	pid[0].value = 0.0;
	pid[1].name = "i";
	pid[1].value = 0.0;
	pid[2].name = "d";
	pid[2].value = 0.0;

	srv.request.config.doubles = pid;

	for(int i = 0; i < nControllers; i++)
		setPID[i].call(srv);
}