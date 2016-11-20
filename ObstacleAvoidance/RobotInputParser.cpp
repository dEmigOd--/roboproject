/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotInputParser class
//	implements input parameter parsing
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include "RobotInputParser.h"

el::Logger* RobotInputParser::logger = Logger::GetLogger("parameter-parser");

bool CaseInsensitiveStringCompare(const std::string& str1, const std::string& str2)
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

RunningParameters RobotInputParser::ParseInputArguments(int argc, char* argv[])
{
	RunningParameters params;

	for (int currentArgN = 1; currentArgN < argc; ++currentArgN)
	{
/////////////////////// DEBUG PARAMETERS /////////////////////////////////
		if (CaseInsensitiveStringCompare(argv[currentArgN], "--record"))
		{
			params.SetRecordMode();
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--replay"))
		{
			params.SetReplayMode();
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--maxRecord"))
		{
			params.numToRecord = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--debugCameras"))
		{
			params.debugCameras = true;
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--debugMath"))
		{
			params.debugMath = true;
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--dontMove"))
		{
			params.shouldMove = false;
			continue;
		}

//////////////////////////// PRODUCTION PARAMETERS ///////////////////////////////
		if (CaseInsensitiveStringCompare(argv[currentArgN], "--workDir"))
		{
			params.wrkDir = argv[++currentArgN];
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--frequency"))
		{
			params.frequency = std::chrono::milliseconds(atoi(argv[++currentArgN]));
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--speed"))
		{
			params.speed = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--turnMultiplier"))
		{
			params.spinMultiplierOnTurns = atof(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--width"))
		{
			int width = atoi(argv[++currentArgN]);
			params.LEFT_CAMERA_FOV_WIDTH = width;
			params.RIGHT_CAMERA_FOV_WIDTH = width;
			params.LEFT_IMAGE_RESIZED_WIDTH = params.LEFT_CAMERA_FOV_WIDTH / Constants::RESIZE_FACTOR;
			params.RIGHT_IMAGE_RESIZED_WIDTH = params.RIGHT_CAMERA_FOV_WIDTH / Constants::RESIZE_FACTOR;
			logger->trace("Width read: %v", width);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--height"))
		{
			int width = atoi(argv[++currentArgN]);
			params.LEFT_CAMERA_FOV_HEIGHT = width;
			params.RIGHT_CAMERA_FOV_HEIGHT = width;
			params.LEFT_IMAGE_RESIZED_HEIGHT = params.LEFT_CAMERA_FOV_HEIGHT / Constants::RESIZE_FACTOR;
			params.RIGHT_IMAGE_RESIZED_HEIGHT = params.RIGHT_CAMERA_FOV_HEIGHT / Constants::RESIZE_FACTOR;
			logger->trace("Height read: %v", width);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--leftCameraIndex"))
		{
			params.leftCameraIdx = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--rightCameraIndex"))
		{
			params.leftCameraIdx = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--useAmatuerForMatching"))
		{
			params.useOpenCVAlgorihmForMatching = MatchingAlgorithm::OWN_BLOCKMATCHING;
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--blockSize"))
		{
			params.nBlockSize = atoi(argv[++currentArgN]);
			continue;
		}

	}

	return params;
}