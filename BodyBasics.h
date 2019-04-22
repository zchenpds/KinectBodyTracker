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
#include "ceres/ceres.h"
#include <Eigen/StdVector>
#include "TFs.h"
//#include "sophus/se3.hpp"
#include <queue>

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

// templated structure that calculates moving mean and variance
template <typename T>
struct MoveStats {
	using Vector3T = Eigen::Matrix<T, 3, 1>;
	MoveStats(int winSize = 200) :
		m_nWinSize(winSize),
		m_Sum(Vector3T::Zero()),
		m_Mean(Vector3T::Zero()),
		m_SumSq(Vector3T::Zero()) {}

	void add(const Vector3T & newT) {
		m_qData.push(newT);
		m_Sum += newT;
		m_SumSq += (newT.array() * newT.array()).matrix();
		if (m_qData.size() > m_nWinSize) {
			Vector3T oldT = m_qData.front();
			m_Sum -= oldT;
			m_SumSq -= (oldT.array() * oldT.array()).matrix();
			m_qData.pop();
		}
		get_mean(m_Mean);
	}

	void get_mean(Vector3T & mean) {
		if (m_qData.size() < 1) mean = Vector3T::Zero();
		else mean = m_Sum / T(m_qData.size());
	}

	void get_movvar(Vector3T & movvar) {
		if (m_qData.size() < 2) movvar = Vector3T::Zero();
		else {
			double n = m_qData.size();
			movvar = (m_SumSq.array() / T(n) - m_Mean.array() * m_Mean.array()) * T(n / (n - 1));
		}
	}
	const int m_nWinSize;
	Vector3T m_Sum;
	Vector3T m_Mean;
	Vector3T m_SumSq;
	std::queue<Vector3T> m_qData;
};
/*
void testMoveStats() {
	//debug
	MoveStats < Eigen::Vector2d > MS(3);
	Eigen::Vector2d res;
	MS.add(Eigen::Vector2d(1, 1));
	MS.get_movvar(res);
	MS.add(Eigen::Vector2d(1, 2));
	MS.get_movvar(res);
	MS.add(Eigen::Vector2d(1, 3));
	MS.get_movvar(res);
	MS.add(Eigen::Vector2d(1, 4));
	MS.get_movvar(res);
	MS.add(Eigen::Vector2d(1, 5));
	MS.get_movvar(res);
	MS.add(Eigen::Vector2d(1, 6));
	MS.get_movvar(res);
	res;
}
*/

struct CalibCostFunctor {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	CalibCostFunctor(){}
	
	template <typename T>
	bool operator()(T const * const sT_kr, T * sResiduals) const {
		using Vector3T = Eigen::Matrix<T, 3, 1>;
		//Eigen::Map<Sophus::SE3<T> const> const T_kr(sT_kr);
		Eigen::Transform<T, 3, Eigen::Affine> T_kr = Eigen::Translation<T, 3>(sT_kr[0], sT_kr[1], sT_kr[2])
			* Eigen::AngleAxis<T>(sT_kr[3], (Eigen::Matrix<T, 3, 1>() << T(1), T(0), T(0)).finished())
			* Eigen::AngleAxis<T>(sT_kr[4], (Eigen::Matrix<T, 3, 1>() << T(0), T(1), T(0)).finished())
			* Eigen::AngleAxis<T>(sT_kr[5], (Eigen::Matrix<T, 3, 1>() << T(1), T(0), T(0)).finished());

		Eigen::Map<Vector3T> residuals(sResiduals);
		residuals = Vector3T::Zero();
		for (auto const & vecPoints : { vecPointsLA, vecPointsRA }) {
			MoveStats<T> MS;
			Vector3T sumVar = Vector3T::Zero();
			assert(vecT_rw.size() == vecPoints.size());
			for (int i = 0; i < vecT_rw.size(); i++) {
				Vector3T P_w = vecT_rw[i].cast<T>() * T_kr * vecPoints[i].cast<T>();
				Vector3T var;
				MS.add(P_w);
				MS.get_movvar(var);
				sumVar += var;
			}
			residuals += sumVar;
		}
		return true;
	}
	
	//std::vector<Sophus::SE3d> vecT_rw; // transform from robot frame to world frame
	// transform from robot frame to world frame
	std::vector < Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > vecT_rw; 
	// Left ankle joint points in Kinect frame
	std::vector < Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vecPointsLA; 
	// Right ankle joint points in Kinect frame
	std::vector < Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vecPointsRA; 

	int getNumDataPoints() {
		int n1 = vecT_rw.size();
		int n2 = vecPointsLA.size();
		int n3 = vecPointsRA.size();
		assert( n1 == n2 && n2 == n3 );
		return vecT_rw.size();
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
	BodyTracker::TFs		m_TFs;

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
	CalibCostFunctor		m_CalibCostFunctor;

	void                    setParams();
	void					log(bool bHeader = false) override;
	// calibrate() may either be called either in processBody(...) or in the simulator thread.
	void					calibrate(BodyTracker::rcVector3d pointLA_k, BodyTracker::rcVector3d pointRA_k);

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
};

