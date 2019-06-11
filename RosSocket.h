#pragma once
#include <stdio.h>
#include "Config.h"
#include <thread>
#include <chrono>
#include "gait_training_robot/human_skeleton.h"

class RosSocket
{
public:
	RosSocket();
	~RosSocket();
	void threadProc();
	void publishMsgSkeleton(const Joint joints[JointType_Count]);
private:
	ros::NodeHandle			nh;
	char*                   m_pszRosMaster;
	

	// Human joint messages
	gait_training_robot::human_skeleton	m_MsgSkeleton;
	ros::Publisher			m_PubSkeleton;
	std::thread				m_Thread;
	
};

