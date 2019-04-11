#pragma once
#include "Aria.h"

class Laser : public ArLMS2xx
{
private:
	ArSerialConnection m_ArConn;
	ArFunctorC<Laser> m_functorReadingCB;
	int m_nReadingCount;

public:
	Laser(const std::string & strPort);
	~Laser();
	void readingCB();
	int getReadingCount();
};

