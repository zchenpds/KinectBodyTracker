#include "CalibSolver.h"
#include <string>
#include <fstream>

void CsvDataReaderBase::openFile(const char * filePath) {
	m_ifs.open(filePath);
	if (m_ifs.is_open() == false)
		throw std::runtime_error(std::string("Cannot open csv file ") + filePath);
	std::string strHeader;
	std::getline(m_ifs, strHeader);

	std::stringstream ssHeader(strHeader);
	std::string strField;
	int i = 0;
	while (std::getline(ssHeader, strField, ',')) {
		m_mapFieldIndex[strField] = i;
		++i;
	}
}

bool CsvDataReaderBase::getNextRow()
{
	if (m_ifs.is_open() == false) 
		throw std::runtime_error(std::string("Error reading a csv file."));
	if (m_ifs.eof()) 
		return false; // Reached the end of the file

	m_RowData.clear();
	std::string strRow;
	std::getline(m_ifs, strRow);	
	std::stringstream ssRow(strRow);
	std::string strCell;
	while (std::getline(ssRow, strCell, ',')) {
		m_RowData.push_back(std::stod(strCell));
	}
	if (m_RowData.size() < m_mapFieldIndex.size())
		return false; // Reached the end of the file because this row is incomplete

	return true;
}


CsvDataReaderKinect::CsvDataReaderKinect()
{
	std::string strKinectPath;
	Config::Instance()->assign("CalibSolver/KinectCsvFilePath", strKinectPath);
	openFile(strKinectPath.c_str());
}

bool CsvDataReaderKinect::getNextRow(double & tKW)
{
	if (CsvDataReaderBase::getNextRow()) {
		const static int indices[] = { m_mapFieldIndex["tKW"]};
		tKW = m_RowData[indices[0]];
		return true;
	}
	else
		return false; // Reached the end of the file 
}

bool CsvDataReaderKinect::getNextRow(double & tKW, BodyTracker::rVector3d pointLA_k, BodyTracker::rVector3d pointRA_k)
{
	if (CsvDataReaderBase::getNextRow()) {
		const static int indices[] = { m_mapFieldIndex["tKW"],
			m_mapFieldIndex["KLAnkleX"], m_mapFieldIndex["KLAnkleY"], m_mapFieldIndex["KLAnkleZ"],
			m_mapFieldIndex["KRAnkleX"], m_mapFieldIndex["KRAnkleY"], m_mapFieldIndex["KRAnkleZ"] 
		};

		tKW = m_RowData[indices[0]];
		pointLA_k(0) = m_RowData[indices[1]];
		pointLA_k(1) = m_RowData[indices[2]];
		pointLA_k(2) = m_RowData[indices[3]];
		pointRA_k(0) = m_RowData[indices[4]];
		pointRA_k(1) = m_RowData[indices[5]];
		pointRA_k(2) = m_RowData[indices[6]];
		return true;
	}
	else
		return false; // Reached the end of the file 
}


CsvDataReaderRobot::CsvDataReaderRobot():
	m_iTimeDiffWithRobot(0)
{
	std::string strRobotPath;
	Config::Instance()->assign("CalibSolver/RobotCsvFilePath", strRobotPath);
	openFile(strRobotPath.c_str());
}

bool CsvDataReaderRobot::getNextRow(double tKW, BodyTracker::rAffine3d tfRW)
{
	// Get the next row if this is the first time this function is invoked
	if (m_RowData.size() == 0)
		if (!CsvDataReaderBase::getNextRow())
			return false; // Reached the end of the file 
	
	// Get the current time stamp
	const static int indices[] = { m_mapFieldIndex["tRW"], 
		m_mapFieldIndex["x"], m_mapFieldIndex["y"], m_mapFieldIndex["th"],
		m_mapFieldIndex["v"], m_mapFieldIndex["w"],
	};
	double tRW = m_RowData[indices[0]];

	// Get the next row if the current row is too old
	while (tRW < tKW - 50) {
		if (!CsvDataReaderBase::getNextRow())
			return false; // Reached the end of the file 
		tRW = m_RowData[indices[0]];
	}
		

	double x = m_RowData[indices[1]];
	double y = m_RowData[indices[2]];
	double th = m_RowData[indices[3]];
	double v = m_RowData[indices[4]];
	double w = m_RowData[indices[5]];

	double dt = (tKW - 50 - tRW) / 1000.0;
	x += v * cos(th) * dt;
	y += v * sin(th) * dt;
	th += th + w * dt;
	m_iTimeDiffWithRobot = dt;

	tfRW = Eigen::Translation3d(x, y, 0.0f) * Eigen::AngleAxisd(th, Eigen::Vector3d::UnitZ());

	return true;
}





CalibSolver::CalibSolver() :
	m_pCalibCostFunctor(NULL),
	m_iDownsampleFactor(32),
	m_iDownsampleChoice(0)
{
	Config::Instance()->assign("CalibSolver/DownsampleFactor", m_iDownsampleFactor);
	Config::Instance()->assign("CalibSolver/DownsampleChoice", m_iDownsampleChoice);
}


CalibSolver::~CalibSolver()
{
}

void CalibSolver::init()
{
	assert(!m_pCalibCostFunctor);
	m_pCalibCostFunctor = new CalibCostFunctor;
}

void CalibSolver::push_back(BodyTracker::rcAffine3d tfRW, BodyTracker::rcVector3d pointLA_k, BodyTracker::rcVector3d pointRA_k)
{
	assert(m_pCalibCostFunctor);
	static int counter;
	if (counter % m_iDownsampleFactor == m_iDownsampleChoice) {
		m_pCalibCostFunctor->vecT_rw.push_back(tfRW);
		m_pCalibCostFunctor->vecPointsLA.push_back(pointLA_k);
		m_pCalibCostFunctor->vecPointsRA.push_back(pointRA_k);
	}
	++counter;
}

void CalibSolver::clear()
{
	assert(m_pCalibCostFunctor);
	m_pCalibCostFunctor->vecT_rw.clear();
	m_pCalibCostFunctor->vecPointsLA.clear();
	m_pCalibCostFunctor->vecPointsRA.clear();
}

int CalibSolver::getNumDataPoints()
{
	return m_pCalibCostFunctor->getNumDataPoints();
}

std::string CalibSolver::solve(double * initial_params)
{
	assert(m_pCalibCostFunctor);
	google::InitGoogleLogging("log_test");
	google::SetLogDestination(google::GLOG_INFO, ".\\");

	// Build the problem.
	ceres::Problem * pProblem = new ceres::Problem;

	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	ceres::CostFunction* cost_function =
		new ceres::AutoDiffCostFunction<CalibCostFunctor, 3, 6>(m_pCalibCostFunctor);
	pProblem->AddResidualBlock(cost_function, NULL, initial_params);
	//pProblem->SetParameterLowerBound(initial_params, 3, M_PI / 2 - 0.1);
	//pProblem->SetParameterUpperBound(initial_params, 3, M_PI / 2 + 0.1);
	//pProblem->SetParameterLowerBound(initial_params, 4, M_PI / 2 - 0.1);
	//pProblem->SetParameterUpperBound(initial_params, 4, M_PI / 2 + 0.1);

	// Hold constant some parameters because otherwise the problem would be overparameterized and ill-conditioned
	pProblem->SetParameterization(initial_params, new ceres::SubsetParameterization(6, { 0, 1, 2, 5 }));

	// Run the solver!
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;
	ceres::Solve(options, pProblem, &summary);

	// clear the cost functor
	clear();

	std::ofstream ofs("CalibSolverReport.txt", std::ofstream::app);
	ofs << summary.BriefReport() << "\n";
	for (int i = 0; i < 6; i++) {
		ofs << initial_params[i] << ", ";
	}
	ofs << "\n\n";

	delete pProblem; // m_pCalibCostFunctor is destroyed while problem is being destroyed.
	pProblem = 0;
	m_pCalibCostFunctor = 0;

	return summary.BriefReport();
}

std::string CalibSolver::readFromFileAndSolve(double * initial_params)
{
	init();
	// Read data from file
	CsvDataReaderKinect cdrKinect;
	CsvDataReaderRobot cdrRobot;

	BodyTracker::Affine3d tfRW; 
	BodyTracker::Vector3d pointLA_k; 
	BodyTracker::Vector3d pointRA_k;
	double tKW = 0;
	while (cdrKinect.getNextRow(tKW, pointLA_k, pointRA_k)) {
		if (cdrRobot.getNextRow(tKW, tfRW))
			push_back(tfRW, pointLA_k, pointRA_k);
		else
			break;
	}
	
	int n = getNumDataPoints();
	if (n < 50) {
		std::stringstream ss;
		ss << "Only " << n << " data points have been collected, which is too few. Calibration aborting.";
		return ss.str();
	}

	return solve(initial_params);
}

CsvDataKinectRewriter::CsvDataKinectRewriter(const double * params) :
	m_JointDataK("K"),
	m_JointDataW("W"),
	m_pRowData(m_cdrKinect.getRowPointer()),
	m_pMapFieldIndex(m_cdrKinect.getFieldInfoPointer())
{
	m_TFs.tfKRNominal = BodyTracker::SE3d({params[0], params[1], params[2], params[3], params[4], params[5] });
	m_TFs.updateTfKR();
	// open file to write to
	BaseLogger::openDataFile("Kinect");

	const static int indices[] = { m_pMapFieldIndex->find("tK")->second,
		m_pMapFieldIndex->find("tKW")->second};

	// Read files
	BodyTracker::Affine3d tfRW;
	double tKW = 0;
	int counter = 0;
	while (m_cdrKinect.getNextRow(tKW)) {
		// Read robot data
		if (m_cdrRobot.getNextRow(tKW, tfRW)) {
			if (counter == 0)
				log(true); //log header
			// Parse Kinect data
			m_JointDataK.tsKinect = (*m_pRowData)[indices[0]];
			m_JointDataK.tsWindows = (*m_pRowData)[indices[1]];
			m_TFs.updateRW(tfRW); // update the tf with robot state

			int i = 0, k = indices[1] + 1;
			for (auto const &jt : jointTypeMap)
			{
				// Record joint coordinates in Kinect frame
				double x = m_JointDataK.data[i + 0] = (*m_pRowData)[i + k + 0];
				double y = m_JointDataK.data[i + 1] = (*m_pRowData)[i + k + 1];
				double z = m_JointDataK.data[i + 2] = (*m_pRowData)[i + k + 2];

				// Transform the coordinates from the Kinect frame to the world frame
				Eigen::Vector3d KFPoint(x, y, z); // Kinect frame point
				Eigen::Vector3d WFPoint = m_TFs * KFPoint; // Convert it to a world frame point
				m_JointDataW.data[i + 0] = WFPoint(0);
				m_JointDataW.data[i + 1] = WFPoint(1);
				m_JointDataW.data[i + 2] = WFPoint(2);

				i += 3;
			}
			log(); // log data

		}
		else
			break;
		++counter;
	}

	
}

void CsvDataKinectRewriter::log(bool bHeader)
{
	m_Mutex.lock();
	const static int indices[] = { m_pMapFieldIndex->find("tO")->second,
		m_pMapFieldIndex->find("tOW")->second, m_pMapFieldIndex->find("trigger")->second
	};
	// Exactly the same as the body of CBodyBasics::log(bool) 
	conditionalLog("tO", (*m_pRowData)[indices[0]], bHeader);
	conditionalLog("tOW", (*m_pRowData)[indices[1]], bHeader);
	conditionalLog("trigger", (*m_pRowData)[indices[2]], bHeader);
	conditionalLog("tK", m_JointDataK.tsKinect, bHeader);
	conditionalLog("tKW", m_JointDataK.tsWindows, bHeader);

	for (int i = 0; i < JOINT_DATA_SIZE; i++)
		conditionalLog(m_JointDataK.names[i].c_str(), m_JointDataK.data[i], bHeader);

	conditionalLog("tDiffR", m_cdrRobot.getDt(), bHeader);
	for (int i = 0; i < JOINT_DATA_SIZE; i++)
		conditionalLog(m_JointDataW.names[i].c_str(), m_JointDataW.data[i], bHeader);
	logEOL();
	m_Mutex.unlock();
}
