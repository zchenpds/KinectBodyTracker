#include "robot.h"
#include <strsafe.h>
#include "Path.h"

Robot::Robot() :
	BaseLogger("Robot"),
	m_pArRobot(NULL),
	m_pRobotConn(NULL),
	m_pArgs(NULL),
	m_pGyro(NULL),
	m_pParser(NULL),
	m_pActionFollow(NULL),
	m_pActionLimiterForwards(NULL),
	m_hWnd(NULL),
	m_pPath(NULL) // will not be instantiated until the first time setParams() is invoked
{
	ZeroMemory(&m_State, sizeof(m_State));
	m_State.isFollowing = false;
	m_State.isCalibrating = false;
	m_State.tsWindows = -1;
	m_Params.isArActionLimiterForwardsEnabled = true;
	m_Params.controlMode = 0;

	m_Params.desiredDistance = 1.5;
	m_Params.VmDistance = 2.5;
	m_Params.VmHeading = 0;
	m_Params.vScale = 1.2;
	m_Params.wScale = 0.3;
	m_Params.kSatPath = 0.3;
	m_Params.kPath = 1.0;

	resetVmGoal();

	m_ControlCmd.v = 0.0;
	m_ControlCmd.w = 0.0;

	// Initialize some global data
	Aria::init();

	m_pArgs = new ArArgumentBuilder;

	// This object parses program options
	m_pParser = new ArArgumentParser(m_pArgs);

	// Load some default values for command line arguments from the ARIAARGS environment variable.
	m_pParser->loadDefaultArguments();

	m_pArgs->add("-laserPort %s", "COM3");
	m_pArgs->add("-robotPort %s", "COM4");

	// Central object that is an interface to the robot and its integrated
	// devices, and which manages control of the robot by the rest of the program.
	m_pArRobot = new ArRobot();
	m_pSonar = new ArSonarDevice();

	// Object that connects to the robot or simulator using program options
	m_pRobotConn = new ArRobotConnector(m_pParser, m_pArRobot);


	m_pActionFollow = new ActionFollow(this);
	m_pActionLimiterForwards = new ArActionLimiterForwards();

	// log heading of the csv file
	log(true);
}


Robot::~Robot()
{
	m_pArRobot->disableMotors();
	Aria::exit(1);
	delete m_pRobotConn;
	delete m_pSonar;
	delete m_pGyro;
	delete m_pArRobot;
	delete m_pArgs;
	delete m_pParser;
	delete m_pActionFollow;
	delete m_pActionLimiterForwards;
	delete m_pPath;
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


#ifdef ROBOT_USE_ACTIONS
	m_pArRobot->addAction(m_pActionFollow, 52);
	m_pArRobot->addAction(m_pActionLimiterForwards, 40);
#endif // ROBOT_USE_ACTIONS

	// If the robot has an Analog Gyro, this object will activate it, and 
	// if the robot does not automatically use the gyro to correct heading,
	// this object reads data from it and corrects the pose in ArRobot
	//m_pGyro = new ArAnalogGyro(m_pArRobot); // to-do: add a gyro switch in config

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
	pConfig->assign("controlMode", m_Params.controlMode);
	pConfig->assign("desiredDistance", m_Params.desiredDistance);
	pConfig->assign("kSatPath", m_Params.kSatPath);
	pConfig->assign("kPath", m_Params.kPath);

	std::string StrRobotPort;
	pConfig->assign("robotPort", StrRobotPort);
	m_pArgs->add("-robotPort %s", StrRobotPort.c_str());

	pConfig->assign("isArActionLimiterForwardsEnabled", m_Params.isArActionLimiterForwardsEnabled);
	if (m_Params.isArActionLimiterForwardsEnabled) m_pActionLimiterForwards->setParameters();
	else m_pActionLimiterForwards->setParameters(1.0, 1.0, 2000.0, 0.1);
	
	resetVmGoal();

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
		if (pathType.compare("PathEight") == 0)
			m_pPath = new BodyTracker::PathEight();
		else
			throw "Unknown path specified.";
	}

	m_pPath->setParams();
}

void Robot::log(bool bHeader) const
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
	conditionalLog("distVm", m_Params.VmDistance, bHeader);
	conditionalLog("headVm", m_Params.VmHeading, bHeader);
	conditionalLog("vScale", m_Params.vScale, bHeader);
	conditionalLog("wScale", m_Params.wScale, bHeader);
	conditionalLog("distDesired", m_Params.desiredDistance, bHeader);
	logEOL();
}

void Robot::updateState() // To do: add mutex.
{
	if (!m_pArRobot->isConnected()) 
		return;
	//m_pArRobot->lock();

	ArPose Pose;
	Pose = m_pArRobot->getPose();
	m_State.dist += pow(pow(Pose.getX() / 1000.0 - m_State.x, 2) + pow(Pose.getY() / 1000.0 - m_State.y, 2), 0.5);
	m_State.x = Pose.getX() / 1000.0;
	m_State.y = Pose.getY() / 1000.0;
	m_State.th = Pose.getTh() * M_PI / 180.0;
	m_State.v = m_pArRobot->getVel() / 1000.0;
	m_State.w = m_pArRobot->getRotVel() * M_PI / 180.0;
	m_State.batteryVolt = m_pArRobot->getRealBatteryVoltageNow();
	m_State.areMotorsEnabled = m_pArRobot->areMotorsEnabled();
	m_State.tsWindows = GetTickCount64();

	ArTime Time = m_pArRobot->getLastOdometryTime();
	m_State.tsRobot = Time.getSecLL() * 1000ll + Time.getMSecLL();

	m_State.xVm = m_State.x + m_Params.VmDistance * cos(m_State.th + m_Params.VmHeading);
	m_State.yVm = m_State.y + m_Params.VmDistance * sin(m_State.th + m_Params.VmHeading);

	log();
	//m_pArRobot->unlock();
}

pcRobotState Robot::getState()
{
	return &m_State;
}

void Robot::setCmd(float v, float w)
{
	if (!m_pArRobot->isConnected())
		return;
	m_pArRobot->lock();

	m_pArRobot->setVel(v * 1e3);
	m_pArRobot->setRotVel(w * 180 / M_PI);

	m_pArRobot->unlock();
}

bool Robot::toggleFollowing()
{
	if (!m_State.isCalibrating)
	{
		if (!m_State.isFollowing)
			m_State.isFollowing = true;
		else
			m_State.isFollowing = false;
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
	else if (m_Params.controlMode == 1) {
		;
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

void Robot::calcControl(float * pV, float * pW, float * pTh)
{
	if (m_Params.controlMode == 0) {
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
		if (pTh != NULL) *pTh = atan2(yToGoal, xToGoal);
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
	else if (m_Params.controlMode == 1){
		// Follows a path
		double vd = 0.15; // needs rewrite
		geometry_msgs::Pose *pPoseDesired = m_pPath->getPoseOnPath(m_State.dist);
		double thDesired = asin(pPoseDesired->orientation.z) * 2;
		double xError = pPoseDesired->position.x - m_State.x;
		double yError = pPoseDesired->position.y - m_State.y;
		double vx = vd * cos(thDesired) + m_Params.kSatPath * tanh(m_Params.kPath * xError / m_Params.kSatPath);
		double vy = vd * sin(thDesired) + m_Params.kSatPath * tanh(m_Params.kPath * yError / m_Params.kSatPath);
		m_ControlCmd.v = vx * cos(m_State.th) + vy * sin(m_State.th);
		m_ControlCmd.w = -vx * sin(m_State.th) / m_Params.VmDistance + vy * cos(m_State.th) / m_Params.VmDistance;
		if (pV != NULL) *pV = m_ControlCmd.v;
		if (pW != NULL) *pW = m_ControlCmd.w;
	}
}

void Robot::resetVmGoal()
{
	float theta = m_State.th + m_Params.VmHeading;
	m_VisualCmd.xVmGoal = m_State.x + m_Params.VmDistance * cos(theta);
	m_VisualCmd.yVmGoal = m_State.y + m_Params.VmDistance * sin(theta);
}

void Robot::setCalibRobotLogging(bool bCalib)
{
	m_State.isCalibrating = bCalib;
}
