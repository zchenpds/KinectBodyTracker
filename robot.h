#pragma once
#include "Aria.h"
#include "Config.h"
#include "ActionFollow.h"



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
	bool			isFollowing;

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

class Robot
{
protected:
	//Aria stuff
	ArRobot*                    m_pArRobot;
	ArRobotConnector*           m_pRobotConn;
	ArArgumentBuilder*          m_pArgs;
	ArArgumentParser*           m_pParser;
	// My stuff
	RobotState                  m_State;
	VisualCmd					m_VisualCmd; // not initalized
	ControlParams				m_Params; // not initialized
	ActionFollow*				m_pActionFollow;
	HWND						m_hWnd;
public:
public:
	Robot();
	~Robot();
	bool init(HWND hWnd);
	void setParams(Config * pConfig);
	void updateState();
	pcRobotState getState();
	void setCmd(float v, float w); // For now not in use
	void startFollowing();
	void stopFollowing();

	// Kinect data processing thread invokes these overloaded functions
	/* 1. Pass the position of the subject's spine base: 
	/ - x: lateral displacement in Camera frame;
	/ - z: longitudinal displacement in Camera frame. */
	void updateVisualCmd(float x, float z);
	bool isVisualCmdTooOld();

	void calcControl(float * pV, float * pW);
	
};