#pragma once
#include "stdafx.h"
#include "robot.h"
#include <thread>
#include <chrono>
#include "Config.h"
#include "TFs.h"
#include "CalibSolver.h"

class Robot;



class Simulator
{
private:
	Robot*						m_pRobot;
	BodyTracker::TFs			m_TFs;
	BodyTracker::CalibFunctor	m_CalibCbFun;
	int							m_iSpeedUpFactor;
	int							m_iStepLengthRobot;
	int							m_iStepCounterRobot;
	std::thread*				m_pThreadRobot;
	std::thread*				m_pThreadKinect;
	std::thread*				m_pThreadTimer;
	INT64						m_tsWindows;
	bool						m_bEnableSimulatedKinect;
	CalibSolver					m_CalibSolver;
	JointData*					m_pJointDataW;
public:
	Simulator(Robot* pRobot, BodyTracker::CalibFunctor CalibCbFun, JointData * pJD = NULL);
	~Simulator();
	void threadProcRobot();
	void updateState(float v, float w);
	void threadProcKinect();
	void threadProcTimer();
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

