/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotInputParser class
//	implements input parameter parsing
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 10
//
//M*/

#include <algorithm>
#include "Utils.h"
#include "RobotInputParser.h"

el::Logger* RobotInputParser::logger = Logger::GetLogger("parameter-parser");

bool StartsWith(const std::string& str, const std::string& substr)
{
	return (str.substr(0, substr.size()) == substr);
}

RobotInputParser::RobotInputParser(int argc, char* argv[])
{
	logger->info("Parsing parameters");
	for (int currentArgN = 1; currentArgN < argc; )
	{
		std::string data(argv[currentArgN++]);
		if (StartsWith(data, "--"))
		{
			// remove --
			data.erase(0, 2);
			params[data] = argv[currentArgN++];

			logger->debug("\t%v = %v", data, params[data]);
		}
		// otherwise ignore
	}
}

bool RobotInputParser::IsSet(const std::string& parameterName) const
{
	return params.count(parameterName) > 0;
}

void RobotInputParser::SetDefaultValues(const std::map<std::string, std::string>& defaultValues)
{
	logger->info("Parsing default parameters");
	SetValues(defaultValues, false, false);
}

void RobotInputParser::Output(const std::string& message, bool suppressOutput) const
{
	if (suppressOutput)
	{
		logger->verbose(2, message);
	}
	else
	{
		logger->trace(message);
	}
}

void RobotInputParser::SetValues(const std::map<std::string, std::string>& defaultValues, bool overrideExisting, bool suppressOutput)
{
	Output("Setting parameters", suppressOutput);

	for (auto & paramName : defaultValues)
	{
		if (overrideExisting || !IsSet(paramName.first))
		{
			// check out, if key exists not inserted
			auto retValue = params.insert(paramName);

			// i.e. not inserted, cause exists
			if (!retValue.second)
			{
				params[paramName.first] = paramName.second;
			}

			Output("\t" + paramName.first + " = " + paramName.second, suppressOutput);
		}
	}
}

void RobotInputParser::SetValue(const std::string& key, const std::string& value)
{
	std::map<std::string, std::string> values;

	values[key] = value;

	SetValues(values, true, true);
}

//el::Logger* RobotInputParser::logger = Logger::GetLogger("parameter-parser");
//
//bool CaseInsensitiveStringCompare(const std::string& str1, const std::string& str2)
//{
//	if (str1.size() != str2.size())
//	{
//		return false;
//	}
//	for (std::string::const_iterator c1 = str1.begin(), c2 = str2.begin(); c1 != str1.end(); ++c1, ++c2)
//	{
//		if (tolower(*c1) != tolower(*c2))
//		{
//			return false;
//		}
//	}
//	return true;
//}
//
//RunningParameters RobotInputParser::ParseInputArguments(int argc, char* argv[])
//{
//	RunningParameters params;
//
//	for (int currentArgN = 1; currentArgN < argc; ++currentArgN)
//	{
///////////////////////// DEBUG PARAMETERS /////////////////////////////////
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--record"))
//		{
//			params.SetRecordMode();
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--replay"))
//		{
//			params.SetReplayMode();
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--maxRecord"))
//		{
//			params.numToRecord = atoi(argv[++currentArgN]);
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--debugCameras"))
//		{
//			params.debugCameras = true;
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--debugMath"))
//		{
//			params.debugMath = true;
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--dontMove"))
//		{
//			params.shouldMove = false;
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--setHorizonHeight"))
//		{
//			params.heightOfHorizon = atoi(argv[++currentArgN]);
//			params.heightOfHorizonSet = true;
//			continue;
//		}
//
////////////////////////////// PRODUCTION PARAMETERS ///////////////////////////////
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--workDir"))
//		{
//			params.wrkDir = argv[++currentArgN];
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--frequency"))
//		{
//			params.frequency = std::chrono::milliseconds(atoi(argv[++currentArgN]));
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--speed"))
//		{
//			params.speed = atoi(argv[++currentArgN]);
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--turnMultiplier"))
//		{
//			params.spinMultiplierOnTurns = atof(argv[++currentArgN]);
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--width"))
//		{
//			int width = atoi(argv[++currentArgN]);
//			params.LEFT_CAMERA_FOV_WIDTH = width;
//			params.RIGHT_CAMERA_FOV_WIDTH = width;
//			params.LEFT_IMAGE_RESIZED_WIDTH = params.LEFT_CAMERA_FOV_WIDTH / Constants::RESIZE_FACTOR;
//			params.RIGHT_IMAGE_RESIZED_WIDTH = params.RIGHT_CAMERA_FOV_WIDTH / Constants::RESIZE_FACTOR;
//			logger->trace("Width read: %v", width);
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--height"))
//		{
//			int width = atoi(argv[++currentArgN]);
//			params.LEFT_CAMERA_FOV_HEIGHT = width;
//			params.RIGHT_CAMERA_FOV_HEIGHT = width;
//			params.LEFT_IMAGE_RESIZED_HEIGHT = params.LEFT_CAMERA_FOV_HEIGHT / Constants::RESIZE_FACTOR;
//			params.RIGHT_IMAGE_RESIZED_HEIGHT = params.RIGHT_CAMERA_FOV_HEIGHT / Constants::RESIZE_FACTOR;
//			logger->trace("Height read: %v", width);
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--leftCameraIndex"))
//		{
//			params.leftCameraIdx = atoi(argv[++currentArgN]);
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--rightCameraIndex"))
//		{
//			params.rightCameraIdx = atoi(argv[++currentArgN]);
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--useAmatuerForMatching"))
//		{
//			params.useOpenCVAlgorihmForMatching = MatchingAlgorithm::OWN_BLOCKMATCHING;
//			continue;
//		}
//
//		if (CaseInsensitiveStringCompare(argv[currentArgN], "--blockSize"))
//		{
//			params.nBlockSize = atoi(argv[++currentArgN]);
//			continue;
//		}
//
//	}
//
//	return params;
//}