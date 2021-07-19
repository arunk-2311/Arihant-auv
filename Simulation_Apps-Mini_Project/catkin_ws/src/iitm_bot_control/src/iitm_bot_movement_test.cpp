#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "iitm_bot_motion_test");
	ros::NodeHandle node_obj;
	ros::Rate loop_rate(1);

	// Publisher node to publish velocities commnad
	ros::Publisher vel_cmd_pub = node_obj.advertise<geometry_msgs::Twist>("/iitm_bot/cmd_vel", 10);

	while (ros::ok())
	{
		geometry_msgs::Twist vel_cmd;

		vel_cmd.linear.x = 0.2;
		vel_cmd.linear.y = 0;
		vel_cmd.linear.z = 0;
		vel_cmd.angular.x = 0;
		vel_cmd.angular.y = 0;
		vel_cmd.angular.z = 0;
		vel_cmd_pub.publish(vel_cmd);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
