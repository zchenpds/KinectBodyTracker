#include "stdafx.h"
#include "BaseLogger.h"

std::string BaseLogger::s_strDataPath = "";

BaseLogger::BaseLogger(): m_pDataFile(NULL)
{}


BaseLogger::~BaseLogger()
{
	m_pDataFile->close();
	delete m_pDataFile;
}

void BaseLogger::openDataFile(const char * name)
{
	// Open a csv file
	std::string fileNameRobot;
	generateFileName(fileNameRobot, name);
	if (s_strDataPath.back() != '\\')
		s_strDataPath += '\\';
	m_pDataFile = new std::ofstream(s_strDataPath + fileNameRobot, std::ofstream::out); // Open the csv file
	if (m_pDataFile->is_open() == false)
		throw "Cannot open data log file";
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

	time_t rawtime; // the number of seconds elapsed since 1900 at 00:00 UTC
	time(&rawtime); // obtain current time
	struct tm *timeinfo = localtime(&rawtime); // represent current time using struct
	std::stringstream ssFileName; // Construct the name of the csv file
	ssFileName << "data-"
		<< timeinfo->tm_year + 1900 << std::setfill('0')
		<< std::setw(2) << timeinfo->tm_mon + 1
		<< std::setw(2) << timeinfo->tm_mday << "-"
		<< std::setw(2) << timeinfo->tm_hour << "-"
		<< std::setw(2) << timeinfo->tm_min << "-"
		<< std::setw(2) << timeinfo->tm_sec << "-"
		<< suffix << MapCounter[strSuffix] << ".csv";
	dest = ssFileName.str();
}

void BaseLogger::logEOL() const
{
	if (m_pDataFile == NULL || m_pDataFile->is_open() == false) return;
	*m_pDataFile << '\n';
}
