#pragma once
#include <stdio.h>
#include "Config.h"
#include "robot.h"
#include <thread>
#include <chrono>

class RosSocket
{
public:
	RosSocket(Robot * pRobot);
	~RosSocket();
	void threadProc();
private:
	Robot*					m_pRobot;
	ros::NodeHandle nh;
	geometry_msgs::Twist    twist_msg;
	ros::Publisher          cmd_vel_pub;
	char*                   m_pszRosMaster;
	std::thread				m_Thread;
};

