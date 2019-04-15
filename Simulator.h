#pragma once
#include "stdafx.h"
#include "robot.h"
#include <thread>
#include <chrono>

class Robot;

class Simulator
{
private:
	Robot*					m_pRobot;
	std::thread				m_Thread;
public:
	Simulator(Robot* pRobot);
	~Simulator();
	void Simulator::threadProc();
	void updateState(float v, float w);
};

