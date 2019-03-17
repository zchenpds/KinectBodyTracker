#pragma once
#include <map>
#include <string>

typedef std::map<std::string, std::string> ConfigParams;

class Config
{
private:
	ConfigParams   m_mapParams;
	int            m_countUpdates;
public:
	Config(const std::string & fileName = "config.txt");
	~Config();
	void load(const std::string & fileName = "config.txt");

	bool assign(const std::string &strKey, std::string & strValue);
	bool assign(const std::string & strKey, float & fValue);
	bool assign(const std::string & strKey, bool & bValue);

	void resetCounter();
	int getUpdateCount();
};

