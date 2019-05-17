#include "ActionFollow.h"


ActionFollow::ActionFollow(Robot * pRobot) :
	m_pRobot(pRobot),
	m_pSonar(NULL),
	ArAction("Follow"),
	m_hWnd(NULL)
{

}

ActionFollow::~ActionFollow()
{
}

void ActionFollow::init(HWND hWnd)
{
	m_hWnd = hWnd;
}

ArActionDesired *ActionFollow::fire(ArActionDesired currentDesired)
{
	m_ActionDesired.reset();
	m_pRobot->updateState();
	pcRobotState pcState = m_pRobot->getState();
	INT64 tsWindows = GetTickCount64();
	
	float v, w;
	m_pRobot->calcControl(&v, &w);

	if (pcState->mode == OM_Following || pcState->mode == OM_Calibrating || pcState->mode ==  OM_Manual)
	{
		m_ActionDesired.setVel(v * 1000.0);
		m_ActionDesired.setRotVel(w * 180.0 / M_PI*1.25);
		//m_ActionDesired.setHeading(th * 180.0 / M_PI);
	}
	else
	{
		m_ActionDesired.setVel(0.0);
		m_ActionDesired.setRotVel(0.0);
	}
	
	return &m_ActionDesired;
	
}

void ActionFollow::setRobot(ArRobot * pArRobot)
{
	ArAction::setRobot(pArRobot);
	m_pSonar = pArRobot->findRangeDevice("sonar");
	if (!m_pSonar)
	{
		if (m_hWnd)
		{
			int msgboxID = MessageBox(m_hWnd,
				L"ActionFollow: Found no SONAR! Continue anyway?",
				NULL, MB_YESNO | MB_ICONWARNING);
			if (msgboxID == IDNO)
				DestroyWindow(m_hWnd);
		}
		else
			deactivate();
	}
}

void ActionFollow::setParams()
{
	//Config* pConfig = Config::Instance();
}
