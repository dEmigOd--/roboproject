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
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 10
//
//M*/

#include "Common.h"
#include "Logger.h"
#include <map>
#include "Logger.h"
#include "RobotParameters.h"
#include "CastTo.h"

class RobotInputParser
{
	static el::Logger* logger;

	struct comp
	{
		bool operator() (const std::string& lhs, const std::string& rhs) const
		{
			return _stricmp(lhs.c_str(), rhs.c_str()) < 0;
		}
	};

	void ThrowIsNotSet(const std::string& parameterName) const
	{
		if (!IsSet(parameterName))
		{
			throw std::runtime_error("Parameter value not set");
		}
	}

	std::map<std::string, std::string, comp> params;

	void Output(const std::string& message, bool suppressOutput = true) const;

public:
	RobotInputParser(int argc, char* argv[]);

	bool IsSet(const std::string& parameterName) const;

	template<typename T>
	const typename T GetValue(const std::string& parameterName) const
	{
		ThrowIsNotSet(parameterName);

		return CastTo<T>(params.at(parameterName));
	}

	void SetDefaultValues(const std::map<std::string, std::string>& defaultValues);
	void SetValues(const std::map<std::string, std::string>& defaultValues, bool overrideExisting = false, bool suppressOutput = true);
	void SetValue(const std::string& key, const std::string& value);
};

