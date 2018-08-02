#pragma once
#include <map>
#include <string>
class Config
{
private:
	std::map<std::string, std::string> m_mapParams;
public:
	Config(std::string fileName = "config.txt");
	~Config();
	bool assign(const std::string &strKey, std::string &strValue);
};

