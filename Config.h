#pragma once
#include <map>
#include <string>
class Config
{
private:
	std::map<std::string, std::string> m_mapParams;
public:
	Config(const std::string & fileName = "config.txt");
	~Config();
	void load(const std::string & fileName = "config.txt");

	bool assign(const std::string &strKey, std::string & strValue);
	bool assign(const std::string & strKey, float & fValue);
};

