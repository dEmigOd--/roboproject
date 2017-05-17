#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	implements common utils used throughout the solution
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/


#include <iomanip>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

#define NOMINMAX
#include "OpenCv.h"

#define CREATE_ENUM(name, ...) enum class name { __VA_ARGS__, __COUNT}; \
inline std::ostream& operator<<(std::ostream& os, name value) { \
std::string enumName = #name; \
std::string str = #__VA_ARGS__; \
int len = str.length(); \
std::vector<std::string> strings; \
std::ostringstream temp; \
for(int i = 0; i < len; i ++) { \
if(isspace(str[i])) continue; \
        else if(str[i] == ',') { \
        strings.push_back(temp.str()); \
        temp.str(std::string());\
        } \
        else temp<< str[i]; \
} \
strings.push_back(temp.str()); \
os << enumName << "::" << strings[static_cast<int>(value)]; \
return os;} 

template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e)
{
	return static_cast<typename std::underlying_type<E>::type>(e);
}

#define TOHUMANREADABLE(name) #name

inline std::string PadWithZeroes(int num, int n_zero)
{
	std::stringstream ss;
	ss << std::setw(n_zero) << std::setfill('0') << num;
	return ss.str();
}

inline void WriteCSV(std::string filename, const cv::Mat& m)
{
	std::ofstream myfile(filename.c_str());
#if CV_VERSION_MAJOR < 3
	cv::Formatter const * c_formatter(cv::Formatter::get("MATLAB"));
	c_formatter->write(myfile, m);
#else
	myfile << cv::format(m, cv::Formatter::FMT_MATLAB) << std::endl;
#endif
}

inline bool CaseInsensitiveStringCompare(const std::string& str1, const std::string& str2)
{
	if (str1.size() != str2.size())
	{
		return false;
	}
	for (std::string::const_iterator c1 = str1.begin(), c2 = str2.begin(); c1 != str1.end(); ++c1, ++c2)
	{
		if (tolower(*c1) != tolower(*c2))
		{
			return false;
		}
	}
	return true;
}

