#include "stdafx.h"
#include "RosPublisher.h"
#include <string>


RosPublisher::RosPublisher(): 
	cmd_vel_pub("cmd_vel", &twist_msg),
	m_pszRosMaster(NULL)
{
	std::string strRosMaster("192.168.1.152:11411");
	Config* pConfig = Config::Instance();
	pConfig->assign("ros_master", strRosMaster);
	m_pszRosMaster = new char[strRosMaster.length() + 1];
	std::strcpy(m_pszRosMaster, strRosMaster.c_str());
	
	
	//printf("Connecting to server at %s\n", ros_master);
	nh.initNode(m_pszRosMaster);

	//printf("Advertising cmd_vel message\n");
	nh.advertise(cmd_vel_pub);

	//printf("Go robot go!\n");
}


RosPublisher::~RosPublisher()
{
	delete[] m_pszRosMaster;
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