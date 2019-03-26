#pragma once

#include "Aria.h"
#include "Config.h"
#include "robot.h"

class Robot;

class ActionFollow : public ArAction
{
public:
	ActionFollow(Robot * pRobot);
	~ActionFollow();
	void init(HWND hWnd);
	// fire, this is what the resolver calls to figure out what this action wants
	virtual ArActionDesired *ActionFollow::fire(ArActionDesired currentDesired);
	// Sets the robot pointer, also gets the sonar device, or deactivates this action if there is no sonar.
	virtual void setRobot(ArRobot * pArRobot);
	// Updates params
	void setParams();
protected:
	Robot*				m_pRobot;
	//ArRobot*			m_pArRobot;
	ArRangeDevice*      m_pSonar;
	ArActionDesired		m_ActionDesired;
	HWND				m_hWnd;
};

