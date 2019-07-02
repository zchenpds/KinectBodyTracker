#include "stdafx.h"
#include "RosSocket.h"
#include <string>


RosSocket::RosSocket():
	m_pszRosMaster(NULL),
	m_PubSkeleton("/skeleton", &m_MsgSkeleton),
	m_Thread(&RosSocket::threadProc, this)
{	
	std::string strRosMaster("192.168.1.116:11411");
	Config* pConfig = Config::Instance();
	pConfig->assign("ros_master", strRosMaster);
	m_pszRosMaster = new char[strRosMaster.length() + 1];
	std::strcpy(m_pszRosMaster, strRosMaster.c_str());

	//printf("Connecting to server at %s\n", ros_master);
	nh.initNode(m_pszRosMaster);
	
	//if (nh.connected() == false)
	//	throw std::runtime_error("RosSocket initialization failed!\n");

	m_MsgSkeleton.header.frame_id = "/kinect2_link";
	m_MsgSkeleton.header.seq = 0;
	m_MsgSkeleton.point_length = JointType_Count;
	nh.advertise(m_PubSkeleton);
}


RosSocket::~RosSocket()
{
	delete[] m_pszRosMaster;
}


void RosSocket::threadProc()
{
	while (1) {
		// Spin
		nh.spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void RosSocket::publishMsgSkeleton(const Joint joints[JointType_Count])
{
	bool bRosSocketEnabled;
	Config::Instance()->assign("RosSocket/enabled", bRosSocketEnabled);
	if (!bRosSocketEnabled) return;
	m_MsgSkeleton.header.seq++;
	m_MsgSkeleton.header.stamp = nh.now();

	geometry_msgs::Point jointPoints[JointType_Count];
	for (int i = 0; i < JointType_Count; i++) {
		jointPoints[i].x = joints[i].Position.X;
		jointPoints[i].y = joints[i].Position.Y;
		jointPoints[i].z = joints[i].Position.Z;
	}

	m_MsgSkeleton.point = jointPoints;
	m_PubSkeleton.publish(&m_MsgSkeleton);
}
