//------------------------------------------------------------------------------
// <copyright file="BodyBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "SyncSocket.h"
#include "Config.h"
#include "SyncSocket.h"
#include "robot.h"
#include "BaseLogger.h"
#include <array>

const std::map <const JointType, const char * > jointTypeMap = {
	{JointType_KneeLeft, "LKnee"},
	{JointType_AnkleLeft, "LAnkle"},
	{JointType_FootLeft, "LFoot"},
	{JointType_KneeRight, "RKnee"},
	{JointType_AnkleRight, "RAnkle"},
	{JointType_FootRight, "RFoot"} 
};

const int JOINT_DATA_SIZE = jointTypeMap.size() * 3;

struct JointData {
	std::vector<float>			data;
	std::vector<std::string>	names;
	INT64						tsKinect;	// (nTime - m_nStartTime) / 10000
	INT64						tsWindows;	// GetTickCount64()
	INT64						tsWindowsBase;
	std::map < JointType, int > jointIndexMap;

	// constructor
	JointData(const char * prefix = ""):
		tsKinect(0), 
		tsWindows(0),
		tsWindowsBase(GetTickCount64())
	{
		data.reserve(JOINT_DATA_SIZE);
		for (int i = 0; i < JOINT_DATA_SIZE; i++)
			data.push_back(0.0f);
		names.reserve(JOINT_DATA_SIZE);
		for (auto &jt : jointTypeMap)
		{
			jointIndexMap[jt.first] = names.size();
			names.push_back(prefix + std::string(jt.second) + "X");
			names.push_back(prefix + std::string(jt.second) + "Y");
			names.push_back(prefix + std::string(jt.second) + "Z");
		}
	}

	std::array<float, 3> operator[](const JointType type) {
		std::array<float, 3> ret;
		int i = jointIndexMap[type];
		ret[0] = data[i + 0];
		ret[1] = data[i + 1];
		ret[2] = data[i + 2];
		return ret;
	}
};

struct TFs {
	Eigen::Affine3f tfRW, tfKR;
	float tau;
	INT64 timeDiffWithRobot;

	TFs(): tau(-0.05f) {
		tfKR = Eigen::Translation3f(-0.15f, 0.0f, 0.7f) 
			* Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY());
	}

	void updateRW(Robot * pRobot, float tsWindows) {
		RobotState RSEstimated;
		timeDiffWithRobot = pRobot->estimateState(&RSEstimated, tsWindows + tau);
		updateRW(RSEstimated.x, RSEstimated.y, RSEstimated.th);
	}

	void updateRW(float x, float y, float th) {
		tfRW = Eigen::Translation3f(x, y, 0.0f) * Eigen::AngleAxisf(th, Eigen::Vector3f::UnitZ());
	}

	Eigen::Vector3f operator*(Eigen::Ref<Eigen::Vector3f> vec) {
		return tfRW * tfKR * vec;
	}
};

void ErrorExit(LPTSTR lpszFunction)
{
	// Retrieve the system error message for the last-error code

	LPVOID lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError();

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR)&lpMsgBuf,
		0, NULL);

	// Display the error message and exit the process

	lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT,
		(lstrlen((LPCTSTR)lpMsgBuf) + lstrlen((LPCTSTR)lpszFunction) + 40) * sizeof(TCHAR));
	StringCchPrintf((LPTSTR)lpDisplayBuf,
		LocalSize(lpDisplayBuf) / sizeof(TCHAR),
		TEXT("%s failed with error %d: %s"),
		lpszFunction, dw, lpMsgBuf);
	MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK);

	LocalFree(lpMsgBuf);
	LocalFree(lpDisplayBuf);
	ExitProcess(dw);
}


class CBodyBasics : public BaseLogger
{
    static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    CBodyBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CBodyBasics();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
    HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;

    // Current Kinect
    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;

    // Body reader
    IBodyFrameReader*       m_pBodyFrameReader;

    // Direct2D
    ID2D1Factory*           m_pD2DFactory;

    // Body/hand drawing
    ID2D1HwndRenderTarget*  m_pRenderTarget;
    ID2D1SolidColorBrush*   m_pBrushJointTracked;
    ID2D1SolidColorBrush*   m_pBrushJointInferred;
    ID2D1SolidColorBrush*   m_pBrushBoneTracked;
    ID2D1SolidColorBrush*   m_pBrushBoneInferred;
    ID2D1SolidColorBrush*   m_pBrushHandClosed;
    ID2D1SolidColorBrush*   m_pBrushHandOpen;
    ID2D1SolidColorBrush*   m_pBrushHandLasso;

	// Robot drawing
	ID2D1HwndRenderTarget*  m_pRenderTarget2;
	ID2D1SolidColorBrush*   m_pBrushRobotBody;
	ID2D1SolidColorBrush*   m_pBrushRobotWheel;
	ID2D1SolidColorBrush*   m_pBrushRobotBodyPred;
	ID2D1SolidColorBrush*   m_pBrushRobotWheelPred;
	ID2D1SolidColorBrush*   m_pBrushRobotObstacle;
	ID2D1SolidColorBrush*   m_pBrushRobotTraversable;
	ID2D1SolidColorBrush*   m_pBrushJoint2;

	// Rendering options
	bool					m_bSonarRenderingEnabled;
	bool					m_bLaserRenderingEnabled;
	std::string				m_strRenderTarget2Frame; // "world" or "robot".

	//
	SyncSocket*				m_pSyncSocket;
	Robot*					m_pRobot;
	JointData				m_JointDataK; // relative to a Kinect frame
	JointData				m_JointDataW; // relative to a World frame
	TFs						m_TFs;

	// Interface
	HWND					m_hWndButtonFollow;
	HWND					m_hWndButtonCalibrate;
	HWND					m_hWndButtonOpenConfig;
	HWND					m_hWndButtonLoad;
	HWND					m_hWndButtonExit;
	HWND                    m_hWndStatic;

	// ROS Socket
	RosSocket*			m_pRosSocket;

	//
	// Define the states of a finite state machine
	typedef enum _CalibState {
		CS_Inactive = 0,
		CS_CountDown,
		CS_Act1,
		CS_Completed,
		CS_Aborted
	} CalibState;
	CalibState				m_pCalibState;

	void                    setParams();
	void					log(bool bHeader = false) override;
	void					calibrate();

	inline void				onPressingButtonFollow();
	inline void				onPressingButtonCalibrate();
	inline void				updateButtons();
    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();
    
    /// <summary>
    /// Handle new body data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="nBodyCount">body data count</param>
    /// <param name="ppBodies">body data in frame</param>
    /// </summary>
    void                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);

    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    /// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
    /// <param name="bForce">force status update</param>
    bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

    /// <summary>
    /// Ensure necessary Direct2d resources are created
    /// </summary>
    /// <returns>S_OK if successful, otherwise an error code</returns>
    HRESULT EnsureDirect2DResources();
	HRESULT EnsureDirect2DResources2();

    /// <summary>
    /// Dispose Direct2d resources 
    /// </summary>
    void DiscardDirect2DResources();
	void DiscardDirect2DResources2();

    /// <summary>
    /// Converts a body point to screen space
    /// </summary>
    /// <param name="bodyPoint">body point to tranform</param>
    /// <param name="width">width (in pixels) of output buffer</param>
    /// <param name="height">height (in pixels) of output buffer</param>
    /// <returns>point in screen-space</returns>
    D2D1_POINT_2F           BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);

    /// <summary>
    /// Draws a body 
    /// </summary>
    /// <param name="pJoints">joint data</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    void                    DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);

    /// <summary>
    /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
    /// </summary>
    /// <param name="handState">state of the hand</param>
    /// <param name="handPosition">position of the hand</param>
    void                    DrawHand(HandState handState, const D2D1_POINT_2F& handPosition);

    /// <summary>
    /// Draws one bone of a body (joint to joint)
    /// </summary>
    /// <param name="pJoints">joint data</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    /// <param name="joint0">one joint of the bone to draw</param>
    /// <param name="joint1">other joint of the bone to draw</param>
    void                    DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);

	// Draws a robot
	void RenderRobotSurroundings();
	void DrawRobot(int drawType, float opacity = 1.0);

	// Transform the coordinates from the Kinect frame to the World frame.
	void KinectToWorld(const CameraSpacePoint & CSP, Eigen::Ref<Eigen::Vector3f> WFP);

};

