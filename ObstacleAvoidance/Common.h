#pragma once

#include "Utils.h"
#include <chrono>

class Constants
{
public :
	// integral types

	static const int CONVERT_TO_PERCENT = 100;
	static const int MAX_COLOR = 255;

	static const int IMAGE_STRETCH_FACTOR = 1;
	static const int IMAGE_COMPRESS_FACTOR = 1;
	static const int CAMERA_FOV_WIDTH = 160;
	static const int CAMERA_FOV_HEIGHT = 120;
	static const int RESIZE_FACTOR = 1;

	// non-class initializable
	static const std::string MatchesWindowName;
	static const std::string LeftWindowName;
	static const std::string RightWindowName;

	static const std::chrono::duration<long long, std::milli> CAPTURE_FROM_CAMERA_INTERVAL;
	static const double CAMERA_ANGLE_WIDTH_MULTIPLIER; // 42 degrees, half of it 21
	static const double CLOSEST_DETECTABLE_RANGE;
	static const double FARTHEST_DETECTABLE_RANGE;
	static const double OPTICAL_AXIS_DISTANCE; // mm
	static const double CHASSIS_WIDTH; // mm
	static const double WHEEL_WIDTH; // mm

	static const double OBSTACLE_LENGTH; // mm
	static const double OBSTACLE_DISTANCE; // mm

};

class RunningParameters
{
private:
	const static int MAX_RECORD = 65536;
	const static int MAX_ZERO_NUM = 5;

	const static std::string ImagesDirName;
	const static std::string OutputDirName;

	const static std::string BaseName;
	const static std::string ImageExtensionName;
	const static std::string MatExtensionName;
	const static std::string Separator;
	const static std::string LeftSuffix;
	const static std::string RightSuffix;

	mutable int currentImageIndex = 0;

	bool recordMode = false;
	bool replayMode = false;
public:
	bool startedProcessing = false;
	bool shouldMove = true;
	bool debugCameras = false;
	bool debugMath = false;
	int numToRecord = MAX_RECORD;

	int speed = 100;
	double speedUnitInMm = 7.0;

	double spinMultiplierOnTurns = 4.0;

	int AcceptableDepthOffset = 2;

	int leftCameraIdx = 0;
	int rightCameraIdx = 1;

	bool useOpenCVAlgorihmForMatching = true;

	int heightOfHorizon;
	int heightOfHorizonSet = false;

	std::chrono::duration<long long, std::milli> frequency = Constants::CAPTURE_FROM_CAMERA_INTERVAL;

	int LEFT_CAMERA_FOV_WIDTH = Constants::CAMERA_FOV_WIDTH;
	int LEFT_CAMERA_FOV_HEIGHT = Constants::CAMERA_FOV_HEIGHT;
	int RIGHT_CAMERA_FOV_WIDTH = Constants::CAMERA_FOV_WIDTH;
	int RIGHT_CAMERA_FOV_HEIGHT = Constants::CAMERA_FOV_HEIGHT;
	int LEFT_IMAGE_RESIZED_WIDTH = LEFT_CAMERA_FOV_WIDTH / Constants::RESIZE_FACTOR;
	int LEFT_IMAGE_RESIZED_HEIGHT = LEFT_CAMERA_FOV_HEIGHT / Constants::RESIZE_FACTOR;
	int RIGHT_IMAGE_RESIZED_WIDTH = RIGHT_CAMERA_FOV_WIDTH / Constants::RESIZE_FACTOR;
	int RIGHT_IMAGE_RESIZED_HEIGHT = RIGHT_CAMERA_FOV_HEIGHT / Constants::RESIZE_FACTOR;

	std::string wrkDir = "";

	int nBlockSize = 15;

	int GetHorizonHeight() const
	{
		return LEFT_IMAGE_RESIZED_HEIGHT / 2;
	}

	bool IsInRecordMode() const
	{
		return debugCameras && recordMode && !RecordingStopped();
	}

	bool IsInReplayMode() const
	{
		return debugCameras && replayMode;
	}

	bool ShouldProcessVideo() const
	{
		return startedProcessing;
	}

	bool IsInDebugMathMode() const
	{
		return debugMath;
	}

	void SetRecordMode(bool newRecordMode = true)
	{
		recordMode = newRecordMode;
	}

	void SetReplayMode(bool newReplayMode = true)
	{
		replayMode = newReplayMode;
	}

	int PrepareForNextImage() const
	{
		return ++currentImageIndex;
	}

	bool RecordingStopped() const
	{
		return currentImageIndex >= numToRecord;
	}

	bool NoMoreImagesLeft() const
	{
		return RecordingStopped();
	}

	std::string BuildLeftImageName() const
	{
		return BuildImageName(LeftSuffix);
	}

	std::string BuildRightImageName() const
	{
		return BuildImageName(RightSuffix);
	}

	std::string BuildMatrixName(const std::string& wantedName) const
	{
		return wrkDir + Separator + OutputDirName + Separator + wantedName + PadWithZeroes(currentImageIndex, RunningParameters::MAX_ZERO_NUM) + MatExtensionName;
	}
private:
	std::string BuildImageName(const std::string& suffix) const
	{
		return wrkDir + Separator + ImagesDirName + Separator + BaseName + suffix + PadWithZeroes(currentImageIndex, RunningParameters::MAX_ZERO_NUM) + ImageExtensionName;
	}
};

