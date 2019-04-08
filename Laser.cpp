#include "Laser.h"


Laser::Laser(const std::string & strPort) :
	ArLMS2xx(0),
	m_functorReadingCB(this, &Laser::readingCB),
	isReadingReceived(false)
{
	if (m_ArConn.open(strPort.c_str()) != 0) {
		throw std::runtime_error((std::string("Error opening serial port: ") + m_ArConn.getPort()).c_str());
	}

	setDeviceConnection(&m_ArConn);
	setPowerControlled(true);
	chooseAutoBaud("38400");
	chooseDegrees("180");
	chooseIncrement("one");
	runAsync();
	if (!blockingConnect()) {
		throw std::runtime_error((std::string("Error connecting to SICK LMS2xx on:") + m_ArConn.getPort()).c_str());
	}

	// Add laser reading callback
	addReadingCB(&m_functorReadingCB);
}


Laser::~Laser()
{
}

void Laser::readingCB()
{
	//ArLog::log(ArLog::Normal, "SUCCESS!");
	/*
	std::vector<ArPoseWithTime> * pPoseScans;
	pPoseScans = this->getCurrentBufferAsVector();
	for (auto && scan : *pPoseScans) {
		ArLog::log(ArLog::Normal, "(%lf, %lf, %lf);\t", scan.getX(), scan.getY(), scan.getThRad());
	}
	ArLog::log(ArLog::Normal, "\n");
	*/
	isReadingReceived = true;
}
