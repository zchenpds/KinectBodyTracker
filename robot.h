#pragma once
#include "Aria.h"
#include "Config.h"
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
	//Aria stuff
	ArRobot*                    m_pArRobot;
	ArRobotConnector*           m_pRobotConn;
	ArArgumentBuilder*          m_pArgs;
	ArArgumentParser*           m_pParser;
	// My stuff
	RobotState                  m_State;
	bool                        m_bInitSucceeded;
	Config*                     m_pConfig;
	HWND						m_hWnd;
public:
public:
	Robot(Config* pConfig);
	~Robot();
	void setParams();
	bool init(HWND hWnd);
	void updateState();
	pcRobotState getState();
	void setCmd(float v, float w);

};

