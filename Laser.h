#pragma once
#include "Aria.h"

class Laser : public ArLMS2xx
{
private:
	ArSerialConnection m_ArConn;
	ArFunctorC<Laser> m_functorReadingCB;
	bool isReadingReceived;
public:
	Laser(const std::string & strPort);
	~Laser();
	void readingCB();
};

