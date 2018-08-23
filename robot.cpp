#include "robot.h"
#include <strsafe.h>


Robot::Robot():
	m_pArRobot(NULL),
	m_pRobotConn(NULL),
	m_pArgs(NULL),
	m_pParser(NULL),
	m_pActionFollow(NULL),
	m_pActionLimiterForwards(NULL),
	m_hWnd(NULL),
	m_pCalibRobotFile(NULL)
{
	ZeroMemory(&m_State, sizeof(m_State));
	m_State.isFollowing = false;
	m_State.isCalibrating = false;
	m_State.tsWindows = -1;

	m_Params.VmDistance = 2.5;
	m_Params.VmHeading = 0;
	m_Params.vScale = 1.2;
	m_Params.wScale = 0.3;

	float theta = m_State.th + m_Params.VmHeading;
	m_VisualCmd.xVmGoal = m_State.x + m_Params.VmDistance * cos(theta);
	m_VisualCmd.yVmGoal = m_State.y;

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

	// Open a csv file
	std::string fileNameRobot;
	generateFileName(fileNameRobot, "Robot");
	m_pRobotFile = new std::ofstream(fileNameRobot, std::ofstream::out); // Open the csv file
	log(m_pRobotFile, true);
}


Robot::~Robot()
{
	m_pArRobot->disableMotors();
	Aria::exit(1);
	delete m_pRobotConn;
	delete m_pSonar;
	delete m_pArRobot;
	delete m_pArgs;
	delete m_pParser;
	delete m_pActionFollow;
	delete m_pActionLimiterForwards;
	m_pRobotFile->close();
	delete m_pRobotFile;
	
}

bool Robot::init(HWND hWnd)
{
	m_hWnd = hWnd;
	m_pActionFollow->init(hWnd);

	// If the robot has an Analog Gyro, this object will activate it, and 
	// if the robot does not automatically use the gyro to correct heading,
	// this object reads data from it and corrects the pose in ArRobot
	//ArAnalogGyro gyro(m_pArRobot);

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

	// Used to access and process sonar range data
	//ArSonarDevice sonarDev;


	// Attach sonarDev to the robot so it gets data from it.
	m_pArRobot->addRangeDevice(m_pSonar);


#ifdef ROBOT_USE_ACTIONS
	m_pArRobot->addAction(m_pActionFollow, 50);
	m_pArRobot->addAction(m_pActionLimiterForwards, 40);
#endif // ROBOT_USE_ACTIONS

	

	m_pArRobot->enableMotors();

	m_pArRobot->runAsync(true);

	return true; // init is successful
}

void Robot::setParams(Config * pConfig)
{
	pConfig->assign("VmDistance", m_Params.VmDistance);
	pConfig->assign("VmHeading", m_Params.VmHeading);
	pConfig->assign("vScale", m_Params.vScale);
	pConfig->assign("wScale", m_Params.wScale);

	std::string StrRobotPort;
	pConfig->assign("robotPort", StrRobotPort);
	m_pArgs->add("-robotPort %s", StrRobotPort.c_str());

	m_pActionFollow->setParams(pConfig);
}

void Robot::log(std::ofstream * pOfs, bool bHeader)
{
	//m_pArRobot->lock();

	
	if (pOfs == NULL)
		return;
	
	ConditionalLog(pOfs, "tRW", m_State.tsWindows, bHeader);
	ConditionalLog(pOfs, "x", m_State.x, bHeader);
	ConditionalLog(pOfs, "y", m_State.y, bHeader);
	ConditionalLog(pOfs, "th", m_State.th, bHeader);
	ConditionalLog(pOfs, "v", m_State.v, bHeader);
	ConditionalLog(pOfs, "w", m_State.w, bHeader);
	ConditionalLog(pOfs, "batVolt", m_State.batteryVolt, bHeader);
	ConditionalLog(pOfs, "isFollowing", m_State.isFollowing, bHeader);
	ConditionalLog(pOfs, "xVm", m_State.xVm, bHeader);
	ConditionalLog(pOfs, "yVm", m_State.yVm, bHeader);
	ConditionalLog(pOfs, "xVmG", m_VisualCmd.xVmGoal, bHeader);
	ConditionalLog(pOfs, "yVmG", m_VisualCmd.yVmGoal, bHeader);
	ConditionalLog(pOfs, "vD", m_ControlCmd.v, bHeader);
	ConditionalLog(pOfs, "wD", m_ControlCmd.w, bHeader);
	ConditionalLog(pOfs, "dVmG", m_Params.VmDistance, bHeader);
	ConditionalLog(pOfs, "hVmG", m_Params.VmHeading, bHeader);
	ConditionalLog(pOfs, "vScale", m_Params.vScale, bHeader);
	ConditionalLog(pOfs, "wScale", m_Params.wScale, bHeader);
	*pOfs << '\n';
	//m_pArRobot->unlock();
}

void Robot::updateState() // To do: add mutex.
{
	if (!m_pArRobot->isConnected()) 
		return;
	//m_pArRobot->lock();

	ArPose Pose;
	Pose = m_pArRobot->getPose();
	m_State.x = Pose.getX() / 1000.0;
	m_State.y = Pose.getY() / 1000.0;
	m_State.th = Pose.getTh() * M_PI / 180.0;
	m_State.v = m_pArRobot->getVel() / 1000.0;
	m_State.w = m_pArRobot->getRotVel() * M_PI / 180.0;
	m_State.batteryVolt = m_pArRobot->getRealBatteryVoltageNow();
	m_State.areMotorsEnabled = m_pArRobot->areMotorsEnabled();
	m_State.tsWindows = GetTickCount64();

	m_State.xVm = m_State.x + m_Params.VmDistance * cos(m_State.th + m_Params.VmHeading);
	m_State.yVm = m_State.y + m_Params.VmDistance * sin(m_State.th + m_Params.VmHeading);

	log(m_pRobotFile);
	if (m_pCalibRobotFile)
		log(m_pCalibRobotFile);
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
	//m_pArRobot->lock();

	m_VisualCmd.tsWindows = GetTickCount64();
	
	// This condition should be easily satisfied since SIP packets arrive every 100 ms
	//if (m_VisualCmd.tsWindows - m_State.tsWindows < 150)
	{
		float theta = m_State.th + m_Params.VmHeading;
		m_VisualCmd.xVmGoal = m_State.x + z * cos(theta) - x * sin(theta);
		m_VisualCmd.yVmGoal = m_State.y + z * sin(theta) + x * cos(theta);
	}

	//m_pArRobot->unlock();
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
	//m_pArRobot->lock();

	float xToGoal, yToGoal;
	float distanceToGoal, headingOfGoal;
	xToGoal = m_VisualCmd.xVmGoal - m_State.x;
	yToGoal = m_VisualCmd.yVmGoal - m_State.y;

	distanceToGoal = sqrt(pow(xToGoal, 2) + pow(yToGoal, 2)) - m_Params.VmDistance;

	float theta = m_State.th + m_Params.VmHeading;
	headingOfGoal = atan2(yToGoal, xToGoal) - theta;
	while (headingOfGoal > M_PI) headingOfGoal -= 2 * M_PI;
	while (headingOfGoal < -M_PI) headingOfGoal += 2 * M_PI;

	*pV = m_ControlCmd.v = m_Params.vScale * distanceToGoal;
	*pW = m_ControlCmd.w = m_Params.wScale * headingOfGoal;
	*pTh = atan2(yToGoal, xToGoal);

	//m_pArRobot->unlock();
}

void Robot::setCalibRobotLogging(bool bCalib)
{
	if (bCalib)
	{
		// Create calib-Robot file
		if (!m_pCalibRobotFile)
		{
			std::string fileNameRobot;
			generateFileName(fileNameRobot, "calib-Robot");
			m_pCalibRobotFile = new std::ofstream(fileNameRobot, std::ofstream::out); // Open a calib file for writing
			log(m_pCalibRobotFile, true);
		}
	}
	else
	{
		// Release calib-Robot file
		if (m_pCalibRobotFile)
		{
			m_pCalibRobotFile->close();
			delete m_pCalibRobotFile;
			m_pCalibRobotFile = NULL;
		}
	}
	
}




void generateFileName(std::string & dest, const char * suffix)
{
	std::string strSuffix(suffix);
	static std::map<std::string, int> MapCounter;
	if (MapCounter.find(strSuffix) == MapCounter.end())
		MapCounter[strSuffix] = 0;
	else
		MapCounter[strSuffix]++;

	time_t rawtime; // the number of seconds elapsed since 1900 at 00:00 UTC
	time(&rawtime); // obtain current time
	struct tm *timeinfo = localtime(&rawtime); // represent current time using struct
	std::stringstream ssFileName; // Construct the name of the csv file
	ssFileName << "data-"
		<< timeinfo->tm_year + 1900 << std::setfill('0')
		<< std::setw(2) << timeinfo->tm_mon + 1
		<< std::setw(2) << timeinfo->tm_mday << "-"
		<< std::setw(2) << timeinfo->tm_hour << "-"
		<< std::setw(2) << timeinfo->tm_min << "-"
		<< std::setw(2) << timeinfo->tm_sec << "-"
		<< suffix << MapCounter[strSuffix] << ".csv";
	dest = ssFileName.str();
}
