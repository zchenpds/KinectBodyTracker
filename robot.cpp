#include "robot.h"
#include <strsafe.h>


Robot::Robot():
	m_pArRobot(NULL),
	m_pRobotConn(NULL),
	m_pArgs(NULL),
	m_pParser(NULL),
	m_pActionFollow(NULL),
	m_pActionLimiterForwards(NULL),
	m_hWnd(NULL)
{
	ZeroMemory(&m_State, sizeof(m_State));
	m_State.isFollowing = false;

	m_Params.VmDistance = 2.5;
	m_Params.VmHeading = 0;
	m_Params.vScale = 1.2;
	m_Params.wScale = 0.3;

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

	// Object that connects to the robot or simulator using program options
	m_pRobotConn = new ArRobotConnector(m_pParser, m_pArRobot);

	m_pActionFollow = new ActionFollow(this);
	m_pActionLimiterForwards = new ArActionLimiterForwards();
}


Robot::~Robot()
{
	m_pArRobot->disableMotors();
	Aria::exit(1);
	delete m_pRobotConn;
	delete m_pArRobot;
	delete m_pArgs;
	delete m_pParser;
	delete m_pActionFollow;
	delete m_pActionLimiterForwards;
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
	ArSonarDevice sonarDev;


	// Attach sonarDev to the robot so it gets data from it.
	m_pArRobot->addRangeDevice(&sonarDev);

	m_pArRobot->addAction(m_pActionFollow, 50);
	//m_pArRobot->addAction(m_pActionLimiterForwards, 40);

	m_pArRobot->enableMotors();

	m_pArRobot->runAsync(true);

	return true; // init is successful
}

void Robot::setParams(Config * pConfig)
{
	pConfig->assign("VmDistance", m_Params.VmDistance);
	pConfig->assign("vScale", m_Params.vScale);
	pConfig->assign("wScale", m_Params.wScale);
	m_pActionFollow->setParams(pConfig);
}

void Robot::updateState() // To do: add mutex.
{
	if (!m_pArRobot->isConnected()) 
		return;
	m_pArRobot->lock();

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

	m_pArRobot->unlock();
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

void Robot::startFollowing()
{
	m_State.isFollowing = true;
}

void Robot::stopFollowing()
{
	m_State.isFollowing = false;
}

void Robot::updateVisualCmd(float x, float z)
{
	m_pArRobot->lock();

	m_VisualCmd.tsWindows = GetTickCount64();

	// This condition should be easily satisfied since SIP packets arrive every 100 ms
	if (m_VisualCmd.tsWindows - m_State.tsWindows < 150)
	{
		m_VisualCmd.xVmGoal = m_State.x + z * cos(m_State.th) - x * sin(m_State.th);
		m_VisualCmd.yVmGoal = m_State.y + z * sin(m_State.th) + x * cos(m_State.th);
	}

	m_pArRobot->unlock();
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
	m_pArRobot->lock();

	float xToGoal, yToGoal;
	float distanceToGoal, headingOfGoal;
	xToGoal = m_VisualCmd.xVmGoal - m_State.x;
	yToGoal = m_VisualCmd.yVmGoal - m_State.y;

	distanceToGoal = sqrt(pow(xToGoal, 2) + pow(yToGoal, 2)) - m_Params.VmDistance;
	headingOfGoal = atan2(yToGoal, xToGoal) - m_State.th;

	while (headingOfGoal > M_PI) headingOfGoal -= 2 * M_PI;
	while (headingOfGoal < -M_PI) headingOfGoal += 2 * M_PI;

	*pV = m_Params.vScale * distanceToGoal;
	*pW = m_Params.wScale * headingOfGoal;
	*pTh = atan2(yToGoal, xToGoal);

	m_pArRobot->unlock();
}

