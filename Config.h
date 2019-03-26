#pragma once
#include <map>
#include <string>

typedef std::map<std::string, std::string> ConfigParams;

// This class is a singleton class. Its life is for the duration of this application.

class Config
{
private:
	static Config* m_pInstance;
	Config() {}
	Config(Config const&) {}
	Config& operator=(Config const&) {}

	ConfigParams   m_mapParams;
	static int     m_countUpdates;

public:
	static Config* Instance(const std::string & fileName = "config.txt");

	void load(const std::string & fileName = "config.txt");

	bool assign(const std::string &strKey, std::string & strValue);
	bool assign(const std::string & strKey, float & fValue);
	bool assign(const std::string & strKey, double & fValue);
	bool assign(const std::string & strKey, bool & bValue);
	bool assign(const std::string & strKey, int & iValue);

	static void resetCounter();
	static int getUpdateCount();
};

