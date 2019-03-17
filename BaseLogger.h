#pragma once
#include <fstream>

// This is a data logger class
class BaseLogger
{
private:
	std::ofstream *m_pDataFile;
public:
	// Constructor of BaseLogger
	// Argument "name" is suffixed to the file name.
	BaseLogger(const char * name);
	~BaseLogger();
protected:
	// This function must be implemented in the derived class
	// because what to be logged in unkown in this base class.
	// Use conditionalLog(...) and logEOF() to log each variable.
	virtual void log(bool bHeader) const = 0 ;

	// Attach timestamp to the beginning of the file name
	void generateFileName(std::string & dest, const char * suffix);

	// Get ready for logging a new record
	void logEOL() const;

	// Log one item of information.
	// If "bHeader" is true, then log "name"; Otherwise, log "value".
	//template<class T>
	//inline void conditionalLog(char const * name, const T & value, bool bHeader) const;
	template<class T>
	void conditionalLog(char const * name, const T & value, bool bHeader) const
	{
		if (m_pDataFile == NULL || m_pDataFile->is_open() == false) return;
		if (bHeader) *m_pDataFile << name << ',';
		else *m_pDataFile << value << ',';
	}

};

