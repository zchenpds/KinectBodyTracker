#pragma once
#include <stdio.h>
#include "Config.h"
#include "robot.h"

class RosPublisher
{
public:
	RosPublisher(Robot * pRobot);
	~RosPublisher();
	void publish();
private:
	Robot*					m_pRobot;
	ros::NodeHandle nh;
	geometry_msgs::Twist    twist_msg;
	ros::Publisher          cmd_vel_pub;
	char*                   m_pszRosMaster;

};

