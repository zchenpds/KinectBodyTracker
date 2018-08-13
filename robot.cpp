#include "robot.h"
#include <strsafe.h>


Robot::Robot(Config* pConfig):
	m_pArRobot(NULL),
	m_pRobotConn(NULL),
	m_pArgs(NULL),
	m_pParser(NULL),
	m_bInitSucceeded(false),
	m_pConfig(pConfig)
	m_hWnd(NULL)
{
	ZeroMemory(&m_State, sizeof(m_State));

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
}


Robot::~Robot()
{
	m_pArRobot->disableMotors();
	Aria::exit(1);
	delete m_pRobotConn;
	delete m_pArRobot;
	delete m_pArgs;
	delete m_pParser;
}

bool Robot::init(HWND hWnd)
{
	m_hWnd = hWnd;

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
			L"Error connecting to the robot! COM port might be wrong.  Continue anyway?", 
			NULL, MB_YESNO | MB_ICONWARNING);
		if (msgboxID == IDNO)
			DestroyWindow(hWnd);
		return false;
	}
	if (!m_pArRobot->isConnected())
	{
		int msgboxID = MessageBox(hWnd, 
			L"Internal error: robot connector succeeded but ArRobot::isConnected() is false! Continue anyway?", 
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
			L"RosAria: ARIA error parsing ARIA startup parameters! Continue anyway?", 
			NULL, MB_YESNO | MB_ICONWARNING);
		if (msgboxID == IDNO)
			DestroyWindow(hWnd);
		return false;
	}

	// Used to access and process sonar range data
	//ArSonarDevice sonarDev;


	// Attach sonarDev to the robot so it gets data from it.
	//m_pArRobot->addRangeDevice(&sonarDev);

	m_pArRobot->enableMotors();

	m_pArRobot->runAsync(true);

	m_bInitSucceeded = true;
	return true; // init is successful
}

void Robot::setParams()
{

}

void Robot::updateState()
{
	ArPose Pose;
	Pose = m_pArRobot->getPose();
	m_State.x = Pose.getX() / 1000.0;
	m_State.y = Pose.getY() / 1000.0;
	m_State.th = Pose.getTh() * M_PI / 180.0;
	m_State.v = m_pArRobot->getVel() / 1000.0;
	m_State.w = m_pArRobot->getRotVel() * M_PI / 180.0;
	m_State.batteryVolt = m_pArRobot->getRealBatteryVoltageNow();
	m_State.areMotorsEnabled = m_pArRobot->areMotorsEnabled();
}

pcRobotState Robot::getState()
{
	return &m_State;
}

void Robot::setCmd(float v, float w)
{
	if (!m_bInitSucceeded) return;

	m_pArRobot->lock();
	m_pArRobot->setVel(v * 1e3);
	m_pArRobot->setRotVel(w * 180 / M_PI);
	m_pArRobot->unlock();
}
