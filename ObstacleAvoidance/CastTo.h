#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	CastTo class
//	implements casting between 4 possible parameter types
//  better use boost_lexical_cast, but this is overkill for this project
//
//  VS 2015 have an interesting bug, as changing the order of Target, Source on CastTo breaks the compilation
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 10
//
//M*/

#include <string>
#include <sstream>
#include "Common.h"

template<typename Target, typename Source>
class CastToImpl
{
public:
	static Target Call(const Source&);
};

template<typename Source>
class CastToImpl<std::string, Source>
{
public:
	static std::string Call(const Source& source)
	{
		std::ostringstream ostr;
		ostr << source;
		return ostr.str();
	}
};

template<>
class CastToImpl<std::string, bool>
{
public:
	static std::string Call(const bool& source)
	{
		std::ostringstream ostr;
		ostr << std::boolalpha << source;
		return ostr.str();
	}
};

template<>
class CastToImpl<bool, std::string>
{
public:
	static bool Call(const std::string& source)
	{
		return (CaseInsensitiveStringCompare(source, "true"));
	}
};

template<>
class CastToImpl<MatchingAlgorithm, std::string>
{
public:
	static MatchingAlgorithm Call(const std::string& source)
	{
		return MatchingAlgorithmMapper::FromString(source);
	}
};

template<>
class CastToImpl<int, std::string>
{
public:
	static int Call(const std::string& source)
	{
		return atoi(source.c_str());
	}
};

template<>
class CastToImpl<double, std::string>
{
public:
	static double Call(const std::string& source)
	{
		return atof(source.c_str());
	}
};

template<typename Target, typename Source>
Target CastTo(const Source& source)
{
	return CastToImpl<Target, Source>::Call(source);
}

