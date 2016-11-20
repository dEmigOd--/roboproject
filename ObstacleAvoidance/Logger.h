#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Logger class
//	wrapper around easylogger
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/


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
