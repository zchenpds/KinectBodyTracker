//#include "pch.h"
#include "Config.h"

#include <fstream>
#include <string>
#include <sstream>

// Initialization of static variables.
Config* Config::m_pInstance = nullptr;
int Config::m_countUpdates = 0;

/** This function is called to create an instance of the class.
    Calling the constructor publicly is not allowed. The constructor
    is private and is only called by this Instance function.
*/
Config* Config::Instance(const std::string & fileName)
{
	if (!m_pInstance) {
		m_pInstance = new Config;
		m_pInstance->load(fileName);
	}
	return m_pInstance;
}


void Config::load(const std::string & fileName)
{
	// Read config from a parameter file
	std::ifstream ifsConfig(fileName);
	if (!ifsConfig.is_open())
	{
		return;
	}
	std::string strLine;
	while (std::getline(ifsConfig, strLine))
	{
		std::string strKey, strValue;
		std::stringstream ss(strLine);
		if (std::getline(ss, strKey, '='))
			if (std::getline(ss, strValue))
				m_mapParams[strKey] = strValue;
	}
	ifsConfig.close();
}


bool Config::assign(const std::string & strKey, std::string & strValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		if (strValue.compare(m_mapParams[strKey]) != 0)
		{
			m_countUpdates++;
			strValue = m_mapParams[strKey];
		}		
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, float & fValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		float fValueNew = stof(m_mapParams[strKey]);
		if (fValue != fValueNew)
		{
			m_countUpdates++;
			fValue = fValueNew;
		}
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, double & fValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		double fValueNew = stof(m_mapParams[strKey]);
		if (fValue != fValueNew)
		{
			m_countUpdates++;
			fValue = fValueNew;
		}
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, bool & bValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		bool bValueNew = m_mapParams[strKey].compare("1") == 0 || m_mapParams[strKey].compare("true") == 0 ||
			m_mapParams[strKey].compare("True") == 0 || m_mapParams[strKey].compare("TRUE") == 0;
		if (bValue != bValueNew)
		{
			m_countUpdates++;
			bValue = bValueNew;
		}
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, int & iValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		int iValueNew = stoi(m_mapParams[strKey]);
		if (iValue != iValueNew)
		{
			m_countUpdates++;
			iValue = iValueNew;
		}
		return true; // Succeeded in assigning the config parameter
	}
}

void Config::resetCounter()
{
	m_countUpdates = 0;
}

int Config::getUpdateCount()
{
	return m_countUpdates;
}
