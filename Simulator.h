#pragma once
#include "stdafx.h"
#include "robot.h"
#include <thread>
#include <chrono>
#include "Config.h"

class Robot;

class Simulator
{
private:
	Robot*					m_pRobot;
	std::thread				m_Thread;
	int						m_iSpeedUpFactor;
	int					m_iStepLength;
	int						m_iStepCounter;
public:
	Simulator(Robot* pRobot);
	~Simulator();
	void Simulator::threadProc();
	void updateState(float v, float w);
};

