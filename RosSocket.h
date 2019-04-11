#pragma once
#include <stdio.h>
#include "Config.h"
#include "robot.h"
#include <thread>
#include <chrono>
#include "sensor_msgs/LaserScan.h"

class RosSocket
{
public:
	RosSocket(Robot * pRobot);
	~RosSocket();
	void threadProc();
private:
	Robot*					m_pRobot;
	ros::NodeHandle			nh;
	char*                   m_pszRosMaster;
	std::thread				m_Thread;

	// Laser
	sensor_msgs::LaserScan	m_MsgLaser;
	ros::Publisher			m_PubLaser;
	
};

