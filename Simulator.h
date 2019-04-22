#pragma once
#include "stdafx.h"
#include "robot.h"
#include <thread>
#include <chrono>
#include "Config.h"
#include "TFs.h"

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
	BodyTracker::TFs			m_TFs;
	std::string					m_StrKinectDataFilePath;
	KinectDataReader			m_KinectDataReader;
	BodyTracker::CalibFunctor	m_CalibCbFun;
	int							m_iSpeedUpFactor;
	int							m_iStepLengthRobot;
	int							m_iStepCounterRobot;
	std::thread*				m_pThreadRobot;
	std::thread*				m_pThreadKinect;
	std::thread*				m_pThreadTimer;
	INT64						m_tsWindows;
public:
	Simulator(Robot* pRobot, BodyTracker::CalibFunctor CalibCbFun);
	~Simulator();
	void threadProcRobot();
	void updateState(float v, float w);
	void threadProcKinect();
	void threadProcTimer();
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

