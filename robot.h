#pragma once
#include "Aria.h"

typedef struct RobotState_ {
	float       x; // meter
	float       y; // meter
	float       th; // rad
	float       v; // meter/sec
	float       w; // rad/sec
	float       batteryVolt;
	float       areMotorsEnabled;
} RobotState;
typedef const RobotState *pcRobotState;

class Robot
{
private:
	ArRobot*                    m_pArRobot;
	ArRobotConnector*           m_pRobotConn;
	RobotState                  m_State;
	bool                        m_bInitSucceeded;
public:
public:
	Robot();
	~Robot();
	bool init(WCHAR *pszText, int len);
	void updateState();
	pcRobotState getState();
	void setCmd(float v, float w);

};

