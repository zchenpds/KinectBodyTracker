#include "stdafx.h"
#include "RosSocket.h"
#include <string>


RosSocket::RosSocket(Robot * pRobot):
	m_pszRosMaster(NULL),
	m_pRobot(pRobot),
	m_Thread(&RosSocket::threadProc, this),
	m_PubLaser("/scan", &m_MsgLaser)
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

	
	nh.advertise(m_PubLaser);
}


RosSocket::~RosSocket()
{
	delete[] m_pszRosMaster;
}


void RosSocket::threadProc()
{

	if (!m_pRobot) throw std::runtime_error("Robot is not instantiated yet.\n");
	while (1) {
		// Laser
		if (m_pRobot->getLaser() && m_pRobot->getLaser()->getReadingCount() > 0) {
			// Populate message
			//........
			m_PubLaser.publish(&m_MsgLaser);
		}
		// Spin
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		nh.spinOnce();
	}
}