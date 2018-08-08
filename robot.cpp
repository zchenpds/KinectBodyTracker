#include "robot.h"
#include <strsafe.h>


Robot::Robot() :
	m_pArRobot(NULL),
	m_pRobotConn(NULL),
	m_bInitSucceeded(false)
{
	ZeroMemory(&m_State, sizeof(m_State));
}


Robot::~Robot()
{
	m_pArRobot->disableMotors();
	Aria::exit(1);
	delete m_pRobotConn;
	delete m_pArRobot;
}

bool Robot::init(WCHAR *pszText, int len)
{
	// Initialize some global data
	Aria::init();

	ArArgumentBuilder args;

	// This object parses program options
	ArArgumentParser parser(&args);

	// Load some default values for command line arguments from the ARIAARGS environment variable.
	parser.loadDefaultArguments();

	args.add("-laserPort %s", "COM3");
	args.add("-robotPort %s", "COM4");

	// Central object that is an interface to the robot and its integrated
	// devices, and which manages control of the robot by the rest of the program.
	m_pArRobot = new ArRobot();

	// Object that connects to the robot or simulator using program options
	ArRobotConnector * m_pRobotConn = new ArRobotConnector(&parser, m_pArRobot);

	// If the robot has an Analog Gyro, this object will activate it, and 
	// if the robot does not automatically use the gyro to correct heading,
	// this object reads data from it and corrects the pose in ArRobot
	ArAnalogGyro gyro(m_pArRobot);

	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.
	if (!m_pRobotConn->connectRobot())
	{
		// Error connecting:
		StringCchPrintf(pszText, len, L"Error connecting to the robot! COM port might be wrong.");
		return false;
	}
	if (!m_pArRobot->isConnected())
	{
		StringCchPrintf(pszText, len, L"Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
		return false;
	}

	// Connector for laser rangefinders
	//ArLaserConnector laserConnector(&parser, m_pArRobot, m_pRobotConn);

	// Connector for compasses
	//ArCompassConnector compassConnector(&parser);

	// Parse the command line options.
	if (!Aria::parseArgs())
	{
		StringCchPrintf(pszText, len, L"RosAria: ARIA error parsing ARIA startup parameters!");
		return false;
	}

	// Used to access and process sonar range data
	//ArSonarDevice sonarDev;


	// Attach sonarDev to the robot so it gets data from it.
	//m_pArRobot->addRangeDevice(&sonarDev);

	m_pArRobot->enableMotors();

	m_bInitSucceeded = true;
	return true; // init is successful
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
