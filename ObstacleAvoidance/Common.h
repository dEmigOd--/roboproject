#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Common definitions
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 13
//
//M*/


#include <chrono>
#include "Utils.h"

enum MatchingAlgorithm
{
	OPENCV_SGBM,
	OWN_BLOCKMATCHING,
};

class MatchingAlgorithmMapper
{
private:
	static std::map<MatchingAlgorithm, std::string> names;
	static std::map<std::string, MatchingAlgorithm> values;

	static bool initialized;
public:
	static void Initialize()
	{
		if (initialized) return;

#define INSERT_ELEMENT(p) names[p] = #p
		INSERT_ELEMENT(OPENCV_SGBM);
		INSERT_ELEMENT(OWN_BLOCKMATCHING);
#undef INSERT_ELEMENT

		for (auto name : names)
			values[name.second] = name.first;

		initialized = true;
	}

	static std::string ToString(const MatchingAlgorithm& value)
	{
		Initialize();
		return names[value];
	}

	static MatchingAlgorithm FromString(const std::string& name)
	{
		Initialize();
		return values[name];
	}
};

inline std::ostream& operator << (std::ostream& ostr, const MatchingAlgorithm& value)
{
	return ostr << MatchingAlgorithmMapper::ToString(value);
}

class Constants
{
public:
	static const int CONVERT_TO_PERCENT = 100;
};