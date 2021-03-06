#pragma once
#include "Aria.h"
#include "Config.h"
#include "ActionFollow.h"
#include <fstream>
#include "stdafx.h"
#include "BaseLogger.h"
#include "Path.h"
#include <vector>
#include "Laser.h"
#include "Simulator.h"
#include "TFs.h"

class ActionFollow;

enum OperationMode {
	OM_Idle = 0,
	OM_Following = 1,
	OM_Calibrating = 2,
	OM_Manual = 3
};

typedef struct RobotState_ {
	float			x; // meter
	float			y; // meter
	float			th; // rad from -2pi to pi
	float			v; // meter/sec
	float			w; // rad/sec
	float			batteryVolt;
	float			areMotorsEnabled;
	float			dist; // meter

	float			xVm; // Virtual marker's x position in World frame
	float			yVm; // Virtual marker's y position in World frame

	INT64			tsWindows; // return of GetTickCount64()
	INT64			tsRobot; // return of ArRobot::getLastOdometryTime() in milliseconds

	OperationMode	mode;

	// debug
	float			thDesired;
	float			xError;
	float			yError;
	float			vx;
	float			vy;
	float			vNew;
	float			wNew;
	float			xVmDesired;
	float			yVmDesired;
} RobotState;
typedef const RobotState *pcRobotState;

typedef struct VisualCmd_ {
	float			xVmGoal; // Virtual marker's goal position in World frame
	float			yVmGoal; // Virtual marker's goal position in World frame
	INT64			tsWindows; // return of GetTickCount64()
	float			rhoTilde; // The desired value minus the actual value of the human follower's distance
	float			rhoDot; // The rate of change of human follower's distance
	float			psiR; // The angle between the robot heading vector and the vector pointing from the human to the robot
} VisualCmd, *pVisualCmd;

typedef struct ControlParams_ {
	float			VmDistance;	// Virtual marker's distance to the robot, in meters.
	float			VmHeading;	// The heading of the vector pointing from the reference point of
								// the robot to the virtual marker, in radians, 
								// relative to the heading of the robot.
	float			vScale;
	float			wScale;

	float			vMax;
	float			wMax;
	float			kappaMax;
	float			aLateralMax;

	float			desiredDistance;
	int				controlMode; //

	float			kSatPath; // for path following control
	float			kPath;

	float			kSatRho;
	float			kRho;
	float			desiredPathSpeed;	
	
	float			thCorrectionFactor; // odometry/thCorrectionFactor=0.98
	
	float			manualAttenuationFactor;
} ControlParams, *pControlParams;

typedef struct ControlCmd_ {
	float			v;
	float			w;
	float			kappa; // Turning curvature
} ControlCmd;

class Robot : BaseLogger
{
	friend class RosSocket;
	friend class Simulator;
protected:
	//Aria stuff
	ArRobot*                    m_pArRobot;
	ArSonarDevice*				m_pSonar;
	Laser*						m_pLaser;
	ArAnalogGyro*				m_pGyro;
	ArRobotConnector*           m_pRobotConn;
	ArArgumentBuilder*          m_pArgs;
	ArArgumentParser*           m_pParser;
	// My stuff
	RobotState                  m_State;
	VisualCmd					m_VisualCmd; // not initalized
	ControlParams				m_Params; // not initialized
	ControlCmd					m_ControlCmd;
	ActionFollow*				m_pActionFollow;
	std::vector<ArAction*>		m_vecpArActionLimiters;
	HWND						m_hWnd;
	BodyTracker::BasePath*		m_pPath;
	Simulator*					m_pSimulator;
public:
public:
	Robot();
	~Robot();
	bool init(HWND hWnd, BodyTracker::CalibFunctor CalibCbFun, JointData * pJD = NULL);
	void setParams();
	int getControlMode();
	void log(bool bHeader = false) override;
	void updateState();
	pcRobotState getState();
	bool isConnected();

	void setCmdV(float v, float maxV = 1.6);
	void setCmdW(float w, float maxW = 4.8);
	void setCmdKappa(float kappa, float maxKappa = 20);

	void accelerateVBy(float deltaV);
	//void accelerateWBy(float deltaW);
	void increaseKappaBy(float deltaKappa);

	bool toggleFollowing();
	bool toggleCalibration();
	bool toggleManual();

	// Kinect data processing thread invokes these functions
	/* 1. Pass the position of the subject's spine base: 
	/ - x: lateral displacement in Camera frame;
	/ - z: longitudinal displacement in Camera frame. */
	void updateVisualCmd(float x, float z);
	void updateControlParams(float VmDistance, float VmHeading, float vScale, float wScale, float desiredDistance);
	void updateControlParams(const float * params);

	bool isVisualCmdTooOld();

	// This function is invoked by the fire function of ActionFollow in sychronization with the SIP loop.
	void calcControl(float * pV = NULL, float * pW = NULL);

	// Initialize the virtual marker to where it would generate zero robot action/movement
	void resetVisualCmd();

	void recordDesiredPath();

	std::function<void()> SIPcbFun;
	bool predictState(RobotState * prs, float tSec);
	INT64 estimateState(BodyTracker::SE2dts * prs, INT64 tsWindows);

	// Get SONAR/laser pointer
	ArSonarDevice * getSonar();
	Laser * getLaser();

	void setControlMode(int mode);
	void restoreControlMode();
	
};

