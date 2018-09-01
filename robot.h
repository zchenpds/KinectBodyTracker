#pragma once
#include "Aria.h"
#include "Config.h"
#include "ActionFollow.h"
#include <fstream>
#include "stdafx.h"

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
} VisualCmd, *pVisualCmd;

typedef struct ControlParams_ {
	float			VmDistance;	// Virtual marker's distance to the robot, in meters.
	float			VmHeading;	// The heading of the vector pointing from the center of
								// the robot and to the virtual marker, in radians.
	float			vScale;
	float			wScale;
} ControlParams, *pControlParams;

typedef struct ControlCmd_ {
	float			v;
	float			w;
} ControlCmd;

class Robot
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
	std::ofstream*				m_pRobotFile;
	std::ofstream*				m_pCalibRobotFile;
	RobotState                  m_State;
	VisualCmd					m_VisualCmd; // not initalized
	ControlParams				m_Params; // not initialized
	ControlCmd					m_ControlCmd;
	ActionFollow*				m_pActionFollow;
	ArActionLimiterForwards*    m_pActionLimiterForwards;
	HWND						m_hWnd;
public:
public:
	Robot();
	~Robot();
	bool init(HWND hWnd);
	void setParams(Config * pConfig);
	void log(std::ofstream * pOfs, bool bHeader = false);
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
	void calcControl(float * pV, float * pW, float * pTh);

	void setCalibRobotLogging(bool bCalib);
	
};


#include <iomanip>
void generateFileName(std::string & dest, const char * suffix = "");
