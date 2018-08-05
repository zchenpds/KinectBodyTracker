#include "Config.h"

#include <fstream>
#include <string>
#include <sstream>



Config::Config(const std::string & fileName)
{
	load(fileName);
}


Config::~Config()
{
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
	std::map<std::string, std::string>::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		strValue = m_mapParams[strKey];
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, float & fValue)
{
	std::map<std::string, std::string>::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		fValue = stof(m_mapParams[strKey]);
		return true; // Succeeded in assigning the config parameter
	}
}
