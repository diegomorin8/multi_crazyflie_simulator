#include "cf_dynamical_model.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
	// ROS set-ups:
	ros::init(argc, argv, "dynamic_model", ros::init_options::AnonymousName); //node name
		self.topic = rospy.get_param()
	ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

	std::string topic; 
	nh.getParam("~topic", topic);

	ROS_INFO("Creating dynamic model for " + topic);
	CF_model cf_model(&nh, topic);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

	ROS_INFO("Initializing simulation for: " + topic);

	cf_model.run(); 
	ros::spin();
	return 0;
}