#pragma once
#include <stdafx.h>
#include <array>
#include "Config.h"

namespace BodyTracker {
	typedef Eigen::Vector3d Vector3d;
	//typedef const Eigen::Ref<const Eigen::Vector3d>& rcVector3d;
	typedef Eigen::Vector3d & rVector3d;
	typedef const Eigen::Vector3d & rcVector3d;
	typedef std::function<void(rcVector3d, rcVector3d)> CalibFunctor;

	typedef Eigen::Affine3d Affine3d;
	typedef Eigen::Affine3d & rAffine3d;
	typedef const Eigen::Affine3d & rcAffine3d;

	// Implementation of the 3D Special Eucleadian Group
	struct SE3d {
		Eigen::Vector3d pos; // translation vector
		Eigen::Vector3d eul; // Euler angles, x, y, x
		SE3d(std::array<double, 6> arr) :
			pos({ arr[0], arr[1], arr[2] }),
			eul({ arr[3], arr[4], arr[5] })
		{}
		SE3d operator+(const SE3d & rhs) {
			SE3d ret({});
			ret.pos = pos + rhs.pos;
			ret.eul = eul + rhs.eul;
			return ret;
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	// Implementation of the 2D Special Eucleadian Group
	struct SE2d {
		double x; // translation x
		double y; // translation y
		double th; // orientation th
		SE2d(std::array<double, 3> arr) :
			x(arr[0]),
			y(arr[1]),
			th(arr[2])
		{}
	};
	
	struct SE2dts : SE2d {
		INT64 tsWindows;
		SE2dts(std::array<double, 3> arr, INT64 ts) :
			SE2d(arr),
			tsWindows(ts)
		{}
	};

	typedef std::function<INT64(BodyTracker::SE2dts * prs, INT64 tsWindows)> functorEstimateRobotState_t;

	// A combination of the transformation from Robot to World and 
	// the transformation from Kinect to robot.
	struct TFs {
		Eigen::Affine3d tfRW; // Pre-multiply tfRW with vector in robot frame to convert it to world frame vector
		Eigen::Affine3d tfKR; // Pre-multiply tfKR with vector in Kinect frame to convert it to robot frame vector
		double tau; // in milliseconds
		INT64 timeDiffWithRobot;

		SE3d tfKRNominal, tfKRCorrection; //nominal value and additive correction for constructing tfKR

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		TFs(functorEstimateRobotState_t functorEstimateRobotState = nullptr) :
			tfRW(Eigen::Affine3f::Identity()),
			tfKR(Eigen::Affine3f::Identity()),
			tau(-50.0),
			tfKRNominal({ -0.15f, 0.0f, 0.7f, M_PI / 2, -M_PI / 2, 0.0f }),
			tfKRCorrection({}),
			m_functorEstimateRobotState(functorEstimateRobotState)
		{
			setParams();
		}

		void setFunctorEstimateRobotState(functorEstimateRobotState_t functorEstimateRobotState) {
			m_functorEstimateRobotState = functorEstimateRobotState;
		}

		void updateRW(INT64 tsWindows) {
			SE2dts poseEstimated({}, 0);
			if (m_functorEstimateRobotState)
				timeDiffWithRobot = m_functorEstimateRobotState(&poseEstimated, tsWindows + (INT64)tau);
			updateRW(poseEstimated.x, poseEstimated.y, poseEstimated.th);
		}

		void updateRW(double x, double y, double th) {
			tfRW = Eigen::Translation3d(x, y, 0.0f) * Eigen::AngleAxisd(th, Eigen::Vector3d::UnitZ());
		}

		void updateRW(const Eigen::Affine3d & tfRW) {
			this->tfRW = tfRW;
		}

		BodyTracker::Vector3d operator*(BodyTracker::rcVector3d vec) {
			return tfRW * tfKR * vec;
		}

		BodyTracker::Vector3d operator/(BodyTracker::rcVector3d vec) {
			/*
			BodyTracker::Vector3d interm = tfRW.inverse() * vec;
			BodyTracker::Vector3d ret = tfKR.inverse() * interm;
			BodyTracker::Vector3d ret2 = Eigen::Translation3d(0.08, -0.92, -0.17) *
				Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * 
				Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()) * 
				//Eigen::Translation3d(0.08, -0.92, -0.17) *
				interm;
			BodyTracker::Vector3d ret3 = (
				Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
				Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()) * 
				Eigen::Translation3d(-0.08, 0.92, 0.17)).inverse() *
				//Eigen::Translation3d(0.08, -0.92, -0.17) *
				interm;*/
			BodyTracker::Vector3d ret = tfKR.inverse() * tfRW.inverse() * vec;
			return ret;
			//return tfKR.inverse() * tfRW.inverse() * vec;
		}

		void setParams() {
			Config* pConfig = Config::Instance();
			std::string strRobotSel;
			pConfig->assign("RobotSel", strRobotSel);
			pConfig->assign("TFs/tfKR/pos" + strRobotSel, tfKRNominal.pos);
			pConfig->assign("TFs/tfKR/eul" + strRobotSel, tfKRNominal.eul);
			updateTfKR();
		}

		void updateTfKR() {
			SE3d params = tfKRNominal + tfKRCorrection;
			tfKR = Eigen::AngleAxisd(params.eul(0), Eigen::Vector3d::UnitX()) *
				Eigen::AngleAxisd(params.eul(1), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(params.eul(2), Eigen::Vector3d::UnitZ()) *
				Eigen::Translation3d(params.pos);

		}
	private:
		functorEstimateRobotState_t m_functorEstimateRobotState;
	}; // struct TFs

} // namespace BodyTracker



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
	JointData(const char * prefix = "") :
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