#include "robot.h"
#include <strsafe.h>
#include "Path.h"

Robot::Robot() :
	BaseLogger(),
	m_pArRobot(NULL),
	m_pSonar(NULL),
	m_pLaser(NULL),
	m_pRobotConn(NULL),
	m_pArgs(NULL),
	m_pGyro(NULL),
	m_pParser(NULL),
	m_pActionFollow(NULL),
	m_hWnd(NULL),
	m_pPath(NULL), // will not be instantiated until the first time setParams() is invoked
	m_pSimulator(NULL)
{
	ZeroMemory(&m_State, sizeof(m_State));
	m_State.isFollowing = false;
	m_State.isCalibrating = false;
	m_State.tsWindows = -1;
	m_Params.controlMode = 0;

	m_Params.desiredDistance = 1.5;
	m_Params.VmDistance = 2.5;
	m_Params.VmHeading = 0;
	m_Params.vScale = 1.2;
	m_Params.wScale = 0.3;

	m_Params.vMax = 0.8;
	m_Params.wMax = 2.4;
	m_Params.kappaMax = 10;
	m_Params.aLateralMax = 0.3;

	m_Params.kSatPath = 0.3;
	m_Params.kPath = 1.0;

	m_Params.kSatRho = 0.5;
	m_Params.kRho = 1.0;
	m_Params.desiredPathSpeed = 0.15;

	m_Params.thCorrectionFactor = 1.0f;

	resetVisualCmd();

	m_ControlCmd.v = 0.0;
	m_ControlCmd.w = 0.0;
	m_ControlCmd.kappa = 0.0;

	Config* pConfig = Config::Instance();

	// Initialize some global data
	Aria::init();

	m_pArgs = new ArArgumentBuilder;

	// This object parses program options
	m_pParser = new ArArgumentParser(m_pArgs);

	// Load some default values for command line arguments from the ARIAARGS environment variable.
	m_pParser->loadDefaultArguments();

	//m_pArgs->add("-laserPort %s", "COM3");
	m_pArgs->add("-robotPort %s", "COM4");

	// Central object that is an interface to the robot and its integrated
	// devices, and which manages control of the robot by the rest of the program.
	m_pArRobot = new ArRobot();
	m_pSonar = new ArSonarDevice();

	// Start laser range finder depending on the config
	bool bLaserEnabled = false;
	std::string strLaserPort = "COM3";
	pConfig->assign("laser/enabled", bLaserEnabled);
	pConfig->assign("laser/port", strLaserPort);
	if (bLaserEnabled)
		m_pLaser = new Laser(strLaserPort);

	// Object that connects to the robot or simulator using program options
	m_pRobotConn = new ArRobotConnector(m_pParser, m_pArRobot);


	m_pActionFollow = new ActionFollow(this);
	m_vecpArActionLimiters.push_back(new ArActionLimiterForwards());
	m_vecpArActionLimiters.push_back(new ArActionLimiterRot());
	m_vecpArActionLimiters.push_back(new ArActionLimiterBackwards());

	// log heading of the csv file
	openDataFile("Robot");
	log(true);
}


Robot::~Robot()
{
	m_pArRobot->disableMotors();
	Aria::exit(1);
	delete m_pRobotConn;
	delete m_pSonar;
	delete m_pLaser;
	delete m_pGyro;
	delete m_pArRobot;
	delete m_pArgs;
	delete m_pParser;
	delete m_pActionFollow;
	while (!m_vecpArActionLimiters.empty()) {
		delete m_vecpArActionLimiters.back();
		m_vecpArActionLimiters.pop_back();
	}
	delete m_pPath;
	delete m_pSimulator;
}

bool Robot::init(HWND hWnd)
{
	m_hWnd = hWnd;
	m_pActionFollow->init(hWnd);


	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.
	
	if (!m_pRobotConn->connectRobot())
	{
		// Error connecting:
		int msgboxID = MessageBox(hWnd, 
			L"robot: Error connecting to the robot! COM port might be wrong.  Continue anyway?", 
			NULL, MB_YESNO | MB_ICONWARNING);
		if (msgboxID == IDNO)
			DestroyWindow(hWnd);
		else
			m_pSimulator = new Simulator(this);
		return false;
	}
	if (!m_pArRobot->isConnected())
	{
		int msgboxID = MessageBox(hWnd, 
			L"robot: robot connector succeeded but ArRobot::isConnected() is false! Continue anyway?", 
			NULL, MB_YESNO | MB_ICONWARNING);
		if (msgboxID == IDNO)
			DestroyWindow(hWnd);
		return false;
	}

	// Connector for laser rangefinders
	//ArLaserConnector laserConnector(m_pParser, m_pArRobot, m_pRobotConn);

	// Connector for compasses
	//ArCompassConnector compassConnector(m_pParser);

	// Parse the command line options.
	if (!Aria::parseArgs())
	{
		int msgboxID = MessageBox(hWnd, 
			L"robot: ARIA error parsing ARIA startup parameters! Continue anyway?", 
			NULL, MB_YESNO | MB_ICONWARNING);
		if (msgboxID == IDNO)
			DestroyWindow(hWnd);
		return false;
	}


	// Attach sonarDev to the robot so it gets data from it.
	m_pArRobot->addRangeDevice(m_pSonar);


	m_pArRobot->addAction(m_pActionFollow, 52);
	for (auto&& it : m_vecpArActionLimiters) {
		m_pArRobot->addAction(it, 60);
	}
	

	// If the robot has an Analog Gyro, this object will activate it, and 
	// if the robot does not automatically use the gyro to correct heading,
	// this object reads data from it and corrects the pose in ArRobot
	m_pGyro = new ArAnalogGyro(m_pArRobot); // to-do: add a gyro switch in config

	m_pArRobot->enableMotors();

	m_pArRobot->runAsync(true);

	return true; // init is successful
}

void Robot::setParams()
{
	Config* pConfig = Config::Instance();
	pConfig->assign("VmDistance", m_Params.VmDistance);
	pConfig->assign("VmHeading", m_Params.VmHeading);
	pConfig->assign("vScale", m_Params.vScale);
	pConfig->assign("wScale", m_Params.wScale);
	pConfig->assign("vMax", m_Params.vMax);
	pConfig->assign("wMax", m_Params.wMax);
	pConfig->assign("kappaMax", m_Params.kappaMax);
	pConfig->assign("aLateralMax", m_Params.aLateralMax);
	pConfig->assign("controlMode", m_Params.controlMode);
	pConfig->assign("desiredDistance", m_Params.desiredDistance);
	pConfig->assign("kSatPath", m_Params.kSatPath);
	pConfig->assign("kPath", m_Params.kPath);
	pConfig->assign("kSatRho", m_Params.kSatRho);
	pConfig->assign("kRho", m_Params.kRho);
	pConfig->assign("desiredPathSpeed", m_Params.desiredPathSpeed);
	pConfig->assign("odometry/thCorrectionFactor", m_Params.thCorrectionFactor);

	std::string StrRobotPort;
	pConfig->assign("robotPort", StrRobotPort);
	m_pArgs->add("-robotPort %s", StrRobotPort.c_str());

	resetVisualCmd();

	m_pActionFollow->setParams();

	// Determine the type of path
	std::string pathType("PathEight");
	pConfig->assign("pathType", pathType);
	if (!m_pPath || pathType.compare(m_pPath->getType())!=0) {
		if (m_pPath) {
			// if the current path is different from that specified in Config.txt
			delete m_pPath;
			m_pPath = nullptr;
		}
		if (pathType.compare("PathEight") == 0) {
			m_pPath = new BodyTracker::PathEight();
		}
		else if (pathType.compare("PathU") == 0) {
			m_pPath = new BodyTracker::PathU();
		}
		else {
			throw std::runtime_error("Robot::setParams() failed with error: Unknown path specified.\n\n");
		}
	}

	m_pPath->setParams();
}

int Robot::getControlMode()
{
	return m_Params.controlMode;
}

void Robot::log(bool bHeader)
{
	
	conditionalLog("tR", m_State.tsRobot, bHeader);
	conditionalLog("tRW", m_State.tsWindows, bHeader);
	conditionalLog("x", m_State.x, bHeader);
	conditionalLog("y", m_State.y, bHeader);
	conditionalLog("th", m_State.th, bHeader);
	conditionalLog("v", m_State.v, bHeader);
	conditionalLog("w", m_State.w, bHeader);
	conditionalLog("batVolt", m_State.batteryVolt, bHeader);
	conditionalLog("isFollowing", m_State.isFollowing, bHeader);
	conditionalLog("xVm", m_State.xVm, bHeader);
	conditionalLog("yVm", m_State.yVm, bHeader);
	conditionalLog("dist", m_State.dist, bHeader);
	conditionalLog("xVmG", m_VisualCmd.xVmGoal, bHeader);
	conditionalLog("yVmG", m_VisualCmd.yVmGoal, bHeader);
	conditionalLog("vD", m_ControlCmd.v, bHeader);
	conditionalLog("wD", m_ControlCmd.w, bHeader);
	conditionalLog("kappaD", m_ControlCmd.kappa, bHeader);
	conditionalLog("distVm", m_Params.VmDistance, bHeader);
	conditionalLog("headVm", m_Params.VmHeading, bHeader);
	conditionalLog("vScale", m_Params.vScale, bHeader);
	conditionalLog("wScale", m_Params.wScale, bHeader);
	conditionalLog("distDesired", m_Params.desiredDistance, bHeader);
	conditionalLog("thDesired", m_State.thDesired, bHeader);
	conditionalLog("xError", m_State.xError, bHeader);
	conditionalLog("yError", m_State.yError, bHeader);
	conditionalLog("vx", m_State.vx, bHeader);
	conditionalLog("vy", m_State.vy, bHeader);
	conditionalLog("vNew", m_State.vNew, bHeader);
	conditionalLog("wNew", m_State.wNew, bHeader);
	conditionalLog("xVmD", m_State.xVmDesired, bHeader);
	conditionalLog("yVmD", m_State.yVmDesired, bHeader);
	logEOL();
}

void Robot::updateState() // To do: add mutex.
{
	if (!m_pArRobot->isConnected()) 
		return;
	//m_pArRobot->lock();

	ArPose Pose;
	Pose = m_pArRobot->getPose();
	
	// Find the theta corrected by the multiplicative factor
	static float thOld;
	float thNew = Pose.getTh() * M_PI / 180.0;
	float dTh = thNew - thOld;
	thOld = thNew;
	if (dTh > M_PI) dTh -= 2 * M_PI;
	else if (dTh <= -M_PI) dTh += 2 * M_PI;
	dTh *= m_Params.thCorrectionFactor;
	m_State.th += dTh;
	if (m_State.th > M_PI) m_State.th -= 2 * M_PI;
	else if (m_State.th <= -M_PI) m_State.th += 2 * M_PI;

	// Update x and y
	static float xOld, yOld;
	float xNew = Pose.getX() / 1000.0;
	float yNew = Pose.getY() / 1000.0;
	float dXY = sqrt(pow(xNew - xOld, 2) + pow(yNew - yOld, 2));
	xOld = xNew;
	yOld = yNew;
	if (dXY > 0) {
		dXY *= sgn(m_pArRobot->getVel());
		m_State.x += dXY * cos(m_State.th);
		m_State.y += dXY * sin(m_State.th);
	}

	// Update v and w
	m_State.v = m_pArRobot->getVel() / 1000.0;
	m_State.w = m_pArRobot->getRotVel() * M_PI / 180.0;
	m_State.batteryVolt = m_pArRobot->getRealBatteryVoltageNow();
	m_State.areMotorsEnabled = m_pArRobot->areMotorsEnabled();
	m_State.tsWindows = GetTickCount64();

	ArTime Time = m_pArRobot->getLastOdometryTime();
	INT64 tsRobotNew = Time.getSecLL() * 1000ll + Time.getMSecLL();
	
	float xVmNew = m_State.x + m_Params.VmDistance * cos(m_State.th + m_Params.VmHeading);
	float yVmNew = m_State.y + m_Params.VmDistance * sin(m_State.th + m_Params.VmHeading);

	static bool is_first = true;
	if (is_first) {
		m_pPath->setPathPose(xVmNew, yVmNew, 0);
		resetVisualCmd();
		recordDesiredPath();
		is_first = false;
	}
	else {
		float dxVm = xVmNew - m_State.xVm;
		float dyVm = yVmNew - m_State.yVm;
		float distGuess = m_State.dist + pow(pow(dxVm, 2) + pow(dyVm, 2), 0.5);
		
		if (0 && m_pPath) {
			// Golden-section search for the closest point on the path
			const float guessRange = 0.2f, goldenRatio = 1.618f;
			const float tolerance = 1e-3;
			float a = distGuess - guessRange;
			float b = distGuess + guessRange;
			float c = b - (b - a) / goldenRatio;
			float d = a + (b - a) / goldenRatio;
			while (fabs(c - d) > tolerance) {
				geometry_msgs::Pose * pPoseDesired;
				pPoseDesired = m_pPath->getPoseOnPath(c);
				float fc = sqrt(pow(pPoseDesired->position.x - m_State.xVm, 2) + pow(pPoseDesired->position.y - m_State.yVm, 2));
				pPoseDesired = m_pPath->getPoseOnPath(d);
				float fd = sqrt(pow(pPoseDesired->position.x - m_State.xVm, 2) + pow(pPoseDesired->position.y - m_State.yVm, 2));
				if (fc < fd) b = d;
				else a = c;
				c = b - (b - a) / goldenRatio;
				d = a + (b - a) / goldenRatio;
			}
			distGuess = (b + a) / 2;
		}
		m_State.dist = distGuess;
	}

	m_State.tsRobot = tsRobotNew;
	m_State.xVm = xVmNew;
	m_State.yVm = yVmNew;
	
	log();

	if (SIPcbFun) {
		SIPcbFun();
	}
}

pcRobotState Robot::getState()
{
	return &m_State;
}

bool Robot::isConnected()
{
	return m_pArRobot->isConnected();
}

void Robot::setCmdV(float v, float maxV)
{
	maxV = min(maxV, m_Params.vMax);
	m_ControlCmd.v = saturate(v, -maxV, maxV);
}

void Robot::setCmdW(float w, float maxW)
{
	maxW = min(maxW, m_Params.wMax);
	m_ControlCmd.w = saturate(w, -maxW, maxW);
}

void Robot::setCmdKappa(float kappa, float maxKappa)
{
	maxKappa = min(maxKappa, m_Params.kappaMax);
	m_ControlCmd.kappa = saturate(kappa, -maxKappa, maxKappa);
}

void Robot::accelerateVBy(float deltaV)
{
	// Assume we are in controlMode 3
	float maxV = sqrt(m_Params.aLateralMax / fabs(m_ControlCmd.kappa));
	setCmdV(m_ControlCmd.v + deltaV, maxV);
}

void Robot::increaseKappaBy(float deltaKappa)
{
	// Assume we are in controlMode 3
	float maxKappa = m_Params.aLateralMax / pow(m_ControlCmd.v, 2);
	setCmdKappa(m_ControlCmd.kappa + deltaKappa, maxKappa);
}

bool Robot::toggleFollowing()
{
	if (!m_State.isCalibrating)
	{
		if (!m_State.isFollowing)
			m_State.isFollowing = true;
		else {
			m_State.isFollowing = false;
			m_ControlCmd.v = 0;
			m_ControlCmd.w = 0;
		}
			
		return true; // succeeded
	}
	else
		return false; // failed
}

bool Robot::toggleCalibration()
{
	if (!m_State.isFollowing)
	{
		if (!m_State.isCalibrating)
			m_State.isCalibrating = true;
		else
			m_State.isCalibrating = false;
		return true; // succeeded
	}
	else
		return false; // failed
}

void Robot::updateVisualCmd(float x, float z)
{
	m_VisualCmd.tsWindows = GetTickCount64();

	if (m_Params.controlMode == 0) {
		float theta = m_State.th + m_Params.VmHeading;
		m_VisualCmd.xVmGoal = m_State.xVm + (m_Params.desiredDistance - z) * cos(theta) - x * sin(theta);
		m_VisualCmd.yVmGoal = m_State.yVm + (m_Params.desiredDistance - z) * sin(theta) + x * cos(theta);
	}
	else if (m_Params.controlMode == 2) {
		m_VisualCmd.psiR = atan2(fabs(x), z);
		m_VisualCmd.rhoDot = 0.0f;
		m_VisualCmd.rhoTilde = m_Params.desiredDistance - z;
	}
}

void Robot::updateControlParams(float VmDistance, float VmHeading, float vScale, float wScale)
{
	m_Params.VmDistance = VmDistance;
	m_Params.VmHeading = VmHeading;
	m_Params.vScale = vScale;
	m_Params.wScale = wScale;
}

void Robot::updateControlParams(const float * params)
{
	m_Params.controlMode = 0;
	updateControlParams(params[0], params[1], params[2], params[3]);
}

bool Robot::isVisualCmdTooOld()
{
	INT64 tsWindows = GetTickCount64();
	if (tsWindows - m_VisualCmd.tsWindows > 1000)
		return true;
	else
		return false;
}

void Robot::calcControl(float * pV, float * pW)
{
	if (m_Params.controlMode == 0 || m_State.isCalibrating) {
		// Move to let the virtual marker approach the goal marker.
#if 1
		float xToGoal, yToGoal;
		float distanceToGoal, headingOfGoal;
		const float tolerance = 0.1;
		xToGoal = m_State.xVm - m_VisualCmd.xVmGoal;
		yToGoal = m_State.yVm - m_VisualCmd.yVmGoal;

		distanceToGoal = sqrt(pow(xToGoal, 2) + pow(yToGoal, 2));

		if (distanceToGoal > tolerance) {
			m_ControlCmd.v = m_Params.vScale * (xToGoal * cos(m_State.th) + yToGoal * sin(m_State.th));
			m_ControlCmd.w = m_Params.wScale / m_Params.VmDistance *
				(-xToGoal * sin(m_State.th) + yToGoal * cos(m_State.th));
		}
		else {
			m_ControlCmd.v = 0;
			m_ControlCmd.w = 0;
		}
		if (pV != NULL) *pV = m_ControlCmd.v;
		if (pW != NULL) *pW = m_ControlCmd.w;
#else

		float xToGoal, yToGoal;
		float distanceToGoal, headingOfGoal;
		xToGoal = m_VisualCmd.xVmGoal - m_State.x;
		yToGoal = m_VisualCmd.yVmGoal - m_State.y;

		distanceToGoal = sqrt(pow(xToGoal, 2) + pow(yToGoal, 2)) - m_Params.VmDistance;

		float theta = m_State.th + m_Params.VmHeading;
		headingOfGoal = atan2(yToGoal, xToGoal) - theta;
		while (headingOfGoal > M_PI) headingOfGoal -= 2 * M_PI;
		while (headingOfGoal < -M_PI) headingOfGoal += 2 * M_PI;

		m_ControlCmd.v = m_Params.vScale * distanceToGoal;
		if (pV != NULL) *pV = m_ControlCmd.v;
		m_ControlCmd.w = m_Params.wScale * headingOfGoal;
		if (pW != NULL) *pW = m_ControlCmd.w;
		if (pTh != NULL) *pTh = atan2(yToGoal, xToGoal);

#endif
	}
	else if (m_Params.controlMode == 1 || m_Params.controlMode == 2){
		// Follows a path
		float vd = 0.15, vMaxPath;
		if (m_Params.controlMode == 1) {
			// Constant vd
			vd = m_Params.desiredPathSpeed;
		}
		else if (m_Params.controlMode == 2) {
			// vd is determined by the state of the human follower
			vd += ( m_VisualCmd.rhoDot + m_Params.kSatRho * tanh(m_Params.kRho * m_VisualCmd.rhoTilde / m_Params.kSatRho) )
				/ cos(m_VisualCmd.psiR);
			vd = saturate(vd, 0.0f, 0.7f);
		}

		if (!m_pPath)
			throw std::runtime_error("Path is not yet instantiated.\n\n");
		
		geometry_msgs::Pose *pPoseDesired = m_pPath->getPoseOnPath(m_State.dist, &vMaxPath);
		double thDesired = asin(pPoseDesired->orientation.z) * 2;
		double xError = pPoseDesired->position.x - m_State.xVm;
		double yError = pPoseDesired->position.y - m_State.yVm;
		double vx = vd * cos(thDesired) + m_Params.kSatPath * tanh(m_Params.kPath * xError / m_Params.kSatPath);
		double vy = vd * sin(thDesired) + m_Params.kSatPath * tanh(m_Params.kPath * yError / m_Params.kSatPath);
		float vNew = vx * cos(m_State.th) + vy * sin(m_State.th);
		float wNew = -vx * sin(m_State.th) / m_Params.VmDistance + vy * cos(m_State.th) / m_Params.VmDistance;

		// debug
		m_State.thDesired = thDesired;
		m_State.xError = xError;
		m_State.yError = yError;
		m_State.vx = vx;
		m_State.vy = vy;
		m_State.vNew = vNew;
		m_State.wNew = wNew;
		m_State.xVmDesired = pPoseDesired->position.x;
		m_State.yVmDesired = pPoseDesired->position.y;

		m_ControlCmd.v = saturate(vNew, 0.0f, min(m_Params.vMax, vMaxPath));
		m_ControlCmd.w = wNew / vNew * m_ControlCmd.v;
		if (pV != NULL) *pV = m_ControlCmd.v;
		if (pW != NULL) *pW = m_ControlCmd.w;
	}
	else if (m_Params.controlMode == 3) {
		if (pV != NULL) *pV = m_ControlCmd.v;
		m_ControlCmd.w = m_ControlCmd.v * m_ControlCmd.kappa;
		if (pW != NULL) *pW = m_ControlCmd.w;
	}

}

void Robot::resetVisualCmd()
{
	float theta = m_State.th + m_Params.VmHeading;
	m_VisualCmd.xVmGoal = m_State.x + m_Params.VmDistance * cos(theta);
	m_VisualCmd.yVmGoal = m_State.y + m_Params.VmDistance * sin(theta);

	m_VisualCmd.rhoTilde = 0.0f;
	m_VisualCmd.rhoDot = 0.0f;
	m_VisualCmd.psiR = 0.0f;
}

void Robot::setCalibRobotLogging(bool bCalib)
{
	m_State.isCalibrating = bCalib;
}

void Robot::recordDesiredPath()
{
	if (!m_pPath) throw std::runtime_error("Path not planned. Cannot record desired path.\n\n");
	std::ofstream ofs;
	std::string fileName;
	generateFileName(fileName, "desiredPath.m");
	ofs.open(fileName, std::ofstream::out | std::ofstream::trunc);
	if (ofs.is_open()) {
		ofs << "desired_path = [";
		for (double dist = 0.0; dist < m_pPath->getCircumference(); dist += 0.1) {
			float vMaxPath;
			geometry_msgs::Pose * poseDesired = m_pPath->getPoseOnPath(dist, &vMaxPath);
			ofs << poseDesired->position.x << ", " << poseDesired->position.y << ", " <<
				asin(poseDesired->orientation.z) * 2 << ", " << vMaxPath << "\n";
		}
		ofs << "];";
		ofs.close();
	}
	else {
		throw std::runtime_error("Robot::recordDesiredPath() failed to open\n\n" + fileName);
	}
}

bool Robot::predictState(RobotState * prs, float tSec)
{
	if (m_pPath && (getControlMode() == 1 || getControlMode() == 2)) {
		// Predict by extrapolating the arc length on the path
		geometry_msgs::Pose * pPose;
		float vMaxPath;
		float dx, dy, dRho, dTh;
		
		// Predict the pose expressed in the World Frame
		if (prs->tsRobot == 0) {
			prs->dist += m_State.dist;
			prs->v = m_State.v;
		}
		prs->tsRobot += (INT64)(tSec * 1000); // the number of millisecs into the future
		prs->dist += prs->v * tSec;
		pPose = m_pPath->getPoseOnPath(prs->dist, &vMaxPath);
		prs->v += saturate(vMaxPath - prs->v, -0.3f * tSec, 0.3f * tSec); // aLongitudinalMax = 0.3f;
		
		// Transform the pose to the Robot Frame
		dx = pPose->position.x - m_State.xVm;
		dy = pPose->position.y - m_State.yVm;
		dRho = sqrt(pow(dx, 2) + pow(dy, 2));
		dTh = atan2(dy, dx) - m_State.th;
		prs->x = dRho * cos(dTh);
		prs->y = dRho * sin(dTh);
		prs->th = asin(pPose->orientation.z) * 2 - m_State.th;
	}
	else {
		float x, y, th, v, w;
		// Predict by numerical integration
		if (getControlMode() == 3) {
			x = prs->x;
			y = prs->y;
			th = prs->th;
			v = m_ControlCmd.v;
			w = m_ControlCmd.v * m_ControlCmd.kappa;
		}
#if 0
		else if (getControlMode() == 1) {
			x = prs->x;
			y = prs->y;
			th = prs->th;
			v = m_State.v;
			w = m_State.w;
		}
#endif
		else {
			return false;
		}

		float dt = 0.1;
		for (float t = 0; t < tSec; t += dt) {
			x += v * cos(th) * dt;
			y += v * sin(th) * dt;
			th += w * dt;
		}
		prs->x = x;
		prs->y = y;
		prs->th = th;
	}
	
	return true;
}

INT64 Robot::estimateState(RobotState * prs, INT64 tsWindows)
{
	prs->tsWindows = tsWindows;

	float dt;
	dt = (tsWindows - m_State.tsWindows) / 1000.0;
	prs->x = m_State.x + m_State.v * cos(m_State.th) * dt;
	prs->y = m_State.y + m_State.v * sin(m_State.th) * dt;
	prs->th = m_State.th + m_State.w * dt;

	return tsWindows - m_State.tsWindows;
}

ArSonarDevice * Robot::getSonar()
{
	return m_pSonar;
}

Laser * Robot::getLaser()
{
	return m_pLaser;
}
