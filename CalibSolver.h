#pragma once
#include "stdafx.h"
#include "ceres/ceres.h"
#include "TFs.h"
#include "Eigen/StdVector"
#include <queue>
#include <array>
#include <fstream>
#include "BaseLogger.h"

// templated structure that calculates moving mean and variance
template <typename T>
struct MoveStats {
	using Vector3T = Eigen::Matrix<T, 3, 1>;
	MoveStats(int winSize = 20) :
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
		CalibCostFunctor() {
		Config::Instance()->assign("CalibSolver/MovVarWinSize", iMovVarWinSize);
	}

	template <typename T>
	bool operator()(T const * const sT_kr, T * sResiduals) const {
		using Vector3T = Eigen::Matrix<T, 3, 1>;
		//Eigen::Map<Sophus::SE3<T> const> const T_kr(sT_kr);
		Eigen::Transform<T, 3, Eigen::Affine> T_kr = 
			Eigen::AngleAxis<T>(sT_kr[3], (Eigen::Matrix<T, 3, 1>() << T(1), T(0), T(0)).finished()) *
			Eigen::AngleAxis<T>(sT_kr[4], (Eigen::Matrix<T, 3, 1>() << T(0), T(1), T(0)).finished()) *
			Eigen::AngleAxis<T>(sT_kr[5], (Eigen::Matrix<T, 3, 1>() << T(0), T(0), T(1)).finished()) *
			Eigen::Translation<T, 3>(sT_kr[0], sT_kr[1], sT_kr[2]);

		Eigen::Map<Vector3T> residuals(sResiduals);
		residuals = Vector3T::Zero();
		for (auto const & vecPoints : { vecPointsLA, vecPointsRA }) {
			MoveStats<T> MS(iMovVarWinSize);
			Vector3T sumVar = Vector3T::Zero();
			assert(vecT_rw.size() == vecPoints.size());
			for (int i = 0; i < vecT_rw.size(); i++) {
				Vector3T P_w = vecT_rw[i].cast<T>() * T_kr * vecPoints[i].cast<T>();
				Vector3T var;
				MS.add(P_w);
				MS.get_movvar(var);
				sumVar += var;
			}
			residuals += sumVar / T(vecPoints.size() - iMovVarWinSize);
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

	int iMovVarWinSize = 20;

	int getNumDataPoints() {
		int n1 = vecT_rw.size();
		int n2 = vecPointsLA.size();
		int n3 = vecPointsRA.size();
		assert(n1 == n2 && n2 == n3);
		return vecT_rw.size();
	}
};


// should not be instantiated
class CsvDataReaderBase {
protected:
	std::ifstream m_ifs;
	std::map<std::string, int> m_mapFieldIndex;
	std::vector<double> m_RowData;
public:
	~CsvDataReaderBase() { m_ifs.close(); }
	void openFile(const char * filePath);
	bool getNextRow();
	const std::vector<double> * getRowPointer() { return &m_RowData; }
	const std::map<std::string, int> * getFieldInfoPointer() { return &m_mapFieldIndex; }
};


// Kinect Data Reader
class CsvDataReaderKinect : public CsvDataReaderBase {
public:
	CsvDataReaderKinect();
	bool getNextRow(double & tKW);
	bool getNextRow(double & tKW, BodyTracker::rVector3d pointLA_k, BodyTracker::rVector3d pointRA_k);
};

// Robot Data Reader
class CsvDataReaderRobot : public CsvDataReaderBase {
	int m_iTimeDiffWithRobot;
public:
	CsvDataReaderRobot();
	bool getNextRow(double tKW, BodyTracker::rAffine3d tfRW);
	int getDt() { return m_iTimeDiffWithRobot; }
};


class CalibSolver
{
private:
	// ceres::problem will take ownership of the cost functor and will destroy when an instance of ceres::problem is destroyed
	CalibCostFunctor*			m_pCalibCostFunctor;
	int							m_iDownsampleFactor;
	int							m_iDownsampleChoice;
public:
	CalibSolver();
	~CalibSolver();
	void init();
	void push_back(BodyTracker::rcAffine3d tfRW, BodyTracker::rcVector3d pointLA_k, BodyTracker::rcVector3d pointRA_k);
	void clear();
	int getNumDataPoints();
	std::string solve(double * initial_params);
	std::string readFromFileAndSolve(double * initial_params);
};


// After running the solver, apply the optimal parameters 
// to original Kinect data and get the new joint coordinates
// in the world frame
class CsvDataKinectRewriter : public BaseLogger {
	CsvDataReaderKinect					m_cdrKinect;
	CsvDataReaderRobot					m_cdrRobot;
	JointData							m_JointDataK; // relative to a Kinect frame
	JointData							m_JointDataW; // relative to a World frame
	const std::vector<double> *			m_pRowData;
	const std::map<std::string, int> *	m_pMapFieldIndex;
	BodyTracker::TFs					m_TFs;
public:
	CsvDataKinectRewriter(const double *);
private:
	void log(bool bHeader = false) override;
};
