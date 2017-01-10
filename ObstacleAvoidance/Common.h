#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RunningParameters class
//	implements parameters which could be changed during system run, incl. only on startup
//
//	*	if you want more things to be controlable w/o recompilation
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, January 10
//
//M*/


#include <chrono>

#include "Utils.h"

enum MatchingAlgorithm
{
	OPENCV_SGBM,
	OWN_BLOCKMATCHING,
};

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
	static const int MAX_RECORD = 65536;
	static const int MAX_ZERO_NUM = 5;

	static const std::string ImagesDirName;
	static const std::string OutputDirName;
	static const std::string CalibrationDirName;

	static const std::string CameraCalibrationDataFileName;
	static const std::string CameraCalibrationDataFileExtension;

	static const std::string BaseName;
	static const std::string ImageExtensionName;
	static const std::string MatExtensionName;
	static const std::string Separator;
	static const std::string LeftSuffix;
	static const std::string RightSuffix;

	mutable int currentImageIndex = 0;
	mutable int imagesReadPerIndex = 1;

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

	int leftCameraIdx = 1;
	int rightCameraIdx = 0;

	MatchingAlgorithm useOpenCVAlgorihmForMatching = OPENCV_SGBM;

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

	// pixels at them we'll pre-test disparities to have a clue, what are the ranges at given speed
	int NumWidthPixelsToTest = 40;
	int NumHeightPixelsToTest = 16;
	int DisparitiesToTestRatio = 2;

	std::string wrkDir = "";
	bool smoothTurn = true;

/////// SECTION INHERETED FROM PREVIOUS PROJECT ////////

	int nBlockSize = 9;
	int THRESHOLD;
	int DENSITY;
	int OBST_THRESHOLD;
	int NPTS;
	int EVENT_THRESHOLD;
	double ERROR_THRESHOLD;
	int GAUSS_SIGMA;
	int LAPLACE_KERN;
	int DIST_THRESHOLD;
	int tresh;

////// METHODS /////
	int GetHorizonHeight() const
	{
		return heightOfHorizonSet ? heightOfHorizon - 1 : (LEFT_IMAGE_RESIZED_HEIGHT / 2);
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
		return !IsInReplayMode() || startedProcessing;
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
		int addToIndex = 1;
		if (IsInReplayMode())
		{
			if (--imagesReadPerIndex)
			{
				addToIndex = 0;
			}
			else
			{
				imagesReadPerIndex = 2;
			}
		}

		return (currentImageIndex += addToIndex);
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

	std::string BuildCameraConfigFilename(int cameraId) const
	{
		return wrkDir + Separator + CalibrationDirName + Separator + CameraCalibrationDataFileName + std::to_string(cameraId) + CameraCalibrationDataFileExtension;
	}

private:
	std::string BuildImageName(const std::string& suffix) const
	{
		return wrkDir + Separator + ImagesDirName + Separator + BaseName + suffix + PadWithZeroes(currentImageIndex, RunningParameters::MAX_ZERO_NUM) + ImageExtensionName;
	}
};

