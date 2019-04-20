#pragma once
#include "stdafx.h"
#include "robot.h"
#include <thread>
#include <chrono>
#include "Config.h"

class Robot;

class KinectDataReader {
private:
	bool m_bReady;

public:
	KinectDataReader(): m_bReady(false) {}
	void openFile(const char * filePath) {
		m_bReady = false;
	}
	bool isReady() { return m_bReady; }

};

class Simulator
{
private:
	Robot*						m_pRobot;
	std::string					m_StrKinectDataFilePath;
	KinectDataReader			m_KinectDataReader;
	BodyTracker::CalibFunctor	m_CalibCbFun;
	int							m_iSpeedUpFactor;
	int							m_iStepLength;
	int							m_iStepCounter;
	std::thread*				m_pThreadRobot;
	std::thread*				m_pThreadKinect;
public:
	Simulator(Robot* pRobot, BodyTracker::CalibFunctor CalibCbFun);
	~Simulator();
	void threadProcRobot();
	void updateState(float v, float w);
	void threadProcKinect();
};

