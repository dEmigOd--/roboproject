#pragma once

#include "easylogging++.h"

class Logger
{
public:
	static const std::string CONFIGFILE;
	static const std::string LOGFILE;
	static void Initialize();
	static el::Logger* GetLogger(const std::string& identity);
private:
	Logger();
};
