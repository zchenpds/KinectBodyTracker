#pragma once
#include "Aria.h"
#include "Config.h"
#include "ActionFollow.h"
#include <fstream>
#include "stdafx.h"
#include "BaseLogger.h"
#include "Path.h"

#define ROBOT_USE_ACTIONS
//#define ROBOT_USE_MOTION_COMMAND_FUNCTIONS

class ActionFollow;

typedef struct RobotState_ {
	float			x; // meter
	float			y; // meter
	float			th; // rad
	float			v; // meter/sec
	float			w; // rad/sec
	float			batteryVolt;
	float			areMotorsEnabled;
	float			dist; // meter

	float			xVm; // Virtual marker's x position in World frame
	float			yVm; // Virtual marker's y position in World frame

	INT64			tsWindows; // return of GetTickCount64()
	INT64			tsRobot; // return of ArRobot::getLastOdometryTime() in milliseconds

	bool			isFollowing;
	bool			isCalibrating;
	//ArMutex			mutex;
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

	float			desiredDistance;
	int				controlMode; //

	bool			isArActionLimiterForwardsEnabled;

	float			kSatPath; // for path following control
	float			kPath;

	float			kSatRho;
	float			kRho;
	float			desiredPathSpeed;	
} ControlParams, *pControlParams;

typedef struct ControlCmd_ {
	float			v;
	float			w;
} ControlCmd;

class Robot : BaseLogger
{
protected:
	//Aria stuff
	ArRobot*                    m_pArRobot;
	ArSonarDevice*				m_pSonar;
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
	ArActionLimiterForwards*    m_pActionLimiterForwards;
	HWND						m_hWnd;
	BodyTracker::BasePath*		m_pPath;
public:
public:
	Robot();
	~Robot();
	bool init(HWND hWnd);
	void setParams();
	int getControlMode();
	void log(bool bHeader = false) const override;
	void updateState();
	pcRobotState getState();
	void setCmd(float v, float w);

	bool toggleFollowing();
	bool toggleCalibration();

	// Kinect data processing thread invokes these functions
	/* 1. Pass the position of the subject's spine base: 
	/ - x: lateral displacement in Camera frame;
	/ - z: longitudinal displacement in Camera frame. */
	void updateVisualCmd(float x, float z);
	void updateControlParams(float VmDistance, float VmHeading, float vScale, float wScale);
	void updateControlParams(const float * params);

	bool isVisualCmdTooOld();
	void calcControl(float * pV = NULL, float * pW = NULL, float * pTh = NULL);

	// Initialize the virtual marker to where it would generate zero robot action/movement
	void resetVisualCmd();

	void setCalibRobotLogging(bool bCalib);

	void recordDesiredPath() const;
	
};

