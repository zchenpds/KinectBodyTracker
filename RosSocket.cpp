#include "stdafx.h"
#include "RosSocket.h"
#include <string>


RosSocket::RosSocket(Robot * pRobot):
	cmd_vel_pub("cmd_vel", &twist_msg),
	m_pszRosMaster(NULL),
	m_pRobot(pRobot),
	m_Thread(&RosSocket::threadProc, this)
{	
}


RosSocket::~RosSocket()
{
	delete[] m_pszRosMaster;
}


void RosSocket::threadProc()
{
	std::string strRosMaster("192.168.1.116:11411");
	Config* pConfig = Config::Instance();
	pConfig->assign("ros_master", strRosMaster);
	m_pszRosMaster = new char[strRosMaster.length() + 1];
	std::strcpy(m_pszRosMaster, strRosMaster.c_str());

	//printf("Connecting to server at %s\n", ros_master);
	nh.initNode(m_pszRosMaster);

	//printf("Advertising cmd_vel message\n");
	nh.advertise(cmd_vel_pub);

	if (!m_pRobot) throw std::runtime_error("Robot is not instantiated yet.\n");
	while (1) {
		twist_msg.linear.x = m_pRobot->m_ControlCmd.v;
		twist_msg.linear.y = 0;
		twist_msg.linear.z = 0;
		twist_msg.angular.x = 0;
		twist_msg.angular.y = 0;
		twist_msg.angular.z = m_pRobot->m_ControlCmd.w;
		cmd_vel_pub.publish(&twist_msg);

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		nh.spinOnce();
	}
}