#include "stdafx.h"
#include "BaseLogger.h"

std::string BaseLogger::s_strDataPath = "";
time_t BaseLogger::m_rawtime = 0;
BaseLogger::BaseLogger()
{
	if (m_rawtime == 0)
		time(&m_rawtime); // obtain current time
}


BaseLogger::~BaseLogger()
{
	if (m_DataFile.is_open()) {
		m_DataFile.close();
	}
}

void BaseLogger::openDataFile(const char * name)
{
	// Open a csv file
	std::string fileNameRobot;
	generateFileName(fileNameRobot, name);
	m_DataFile.open(fileNameRobot, std::ofstream::out); // Open the csv file
	if (m_DataFile.is_open() == false)
		throw std::runtime_error("Cannot open file\n" + fileNameRobot +
			"\n\nBaseLogger::openDataFile(const char * name)");
}

void BaseLogger::generateFileName(std::string & dest, const char * suffix)
{
	//system("mkdir data");
	std::string strSuffix(suffix);
	static std::map<std::string, int> MapCounter;
	if (MapCounter.find(strSuffix) == MapCounter.end())
		MapCounter[strSuffix] = 0;
	else
		MapCounter[strSuffix]++;

	struct tm *timeinfo = localtime(&m_rawtime); // represent current time using struct
	std::stringstream ssFileName; // Construct the name of the csv file
	ssFileName << "data_"
		<< timeinfo->tm_year + 1900 << std::setfill('0')
		<< std::setw(2) << timeinfo->tm_mon + 1
		<< std::setw(2) << timeinfo->tm_mday << "_"
		<< std::setw(2) << timeinfo->tm_hour << "_"
		<< std::setw(2) << timeinfo->tm_min << "_"
		<< std::setw(2) << timeinfo->tm_sec << "_"
		<< suffix;
	if (strSuffix.find('.') == std::string::npos)
		ssFileName << MapCounter[strSuffix] << ".csv";
	if (s_strDataPath.back() != '\\')
		s_strDataPath += '\\';
	dest = s_strDataPath + ssFileName.str();
}

void BaseLogger::logEOL()
{
	if (m_DataFile.is_open() == false) return;
	m_DataFile << '\n';
}
