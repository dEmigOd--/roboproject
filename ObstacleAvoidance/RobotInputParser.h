#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotInputParser class
//	implements input parameter parsing
//
//	*	add here coupling between input flags and some parameter you want to control
//	*	some constants could also become parameters !
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include "Common.h"
#include "Logger.h"

class RobotInputParser
{
	static el::Logger* logger;
public:
	static RunningParameters ParseInputArguments(int argc, char* argv[]);
};
