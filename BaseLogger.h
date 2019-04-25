#pragma once
#include <fstream>

// This is a data logger class
class BaseLogger
{
private:
	std::ofstream m_DataFile;
protected:
	static std::string s_strDataPath;
	static time_t m_rawtime; // the number of seconds elapsed since 1900 at 00:00 UTC
public:
	// Constructor of BaseLogger
	// Argument "name" is suffixed to the file name.
	BaseLogger();
	~BaseLogger();

	// This function must be called before log() can be invoked.
	void openDataFile(const char * name);
protected:
	// This function must be implemented in the derived class
	// because what to be logged is unkown in this base class.
	// Use conditionalLog(...) and logEOF() to log each variable.
	virtual void log(bool bHeader) = 0 ;

	// Attach timestamp to the beginning of the file name
	void generateFileName(std::string & dest, const char * suffix);

	// Get ready for logging a new record
	void logEOL();

	// Log one item of information.
	// If "bHeader" is true, then log "name"; Otherwise, log "value".
	// TO-DO: Make bHeader a private member variable that can be set by a member function void setHeader(bool)
	template<class T>
	void conditionalLog(char const * name, const T & value, bool bHeader)
	{
		if (m_DataFile.is_open() == false) return;
		if (bHeader) m_DataFile << name << ',';
		else m_DataFile << value << ',';
	}

};

