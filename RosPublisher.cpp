#include "stdafx.h"
#include "RosPublisher.h"



RosPublisher::RosPublisher(): cmd_vel_pub("cmd_vel", &twist_msg)
{
	char ros_master[] = "192.168.1.110:11411";

	//printf("Connecting to server at %s\n", ros_master);
	nh.initNode(ros_master);

	//printf("Advertising cmd_vel message\n");
	nh.advertise(cmd_vel_pub);

	//printf("Go robot go!\n");
}


RosPublisher::~RosPublisher()
{
}


void RosPublisher::publish(float p[2])
{
	twist_msg.linear.x = p[0];
	twist_msg.linear.y = 0;
	twist_msg.linear.z = 0;
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = p[1];
	cmd_vel_pub.publish(&twist_msg);

	nh.spinOnce();
}