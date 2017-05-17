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
//	2017, May 11
//
//M*/

#include "RobotInputParser.h"

class RunningParameters
{
private:
	mutable int currentImageIndex = 0;
	mutable int imagesReadPerIndex = 1;

public:

	RobotInputParser rip;

	RunningParameters(const RobotInputParser& rip)
		: rip(rip)
	{
	}

	template<typename T>
	const typename T GetValue(const std::string& parameterName) const
	{
		return rip.GetValue<T>(parameterName);
	}

	int heightOfHorizon;
	int heightOfHorizonSet = false;

	std::chrono::duration<long long, std::milli> Frequency() const
	{
		return std::chrono::milliseconds(GetValue<int>(RobotParameters::CAPTURE_FROM_CAMERA_INTERVAL));
	}

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
		return GetValue<bool>(RobotParameters::heightOfHorizonSet) ? 
			GetValue<int>(RobotParameters::heightOfHorizon) - 1 : 
			(GetValue<int>(RobotParameters::LEFT_IMAGE_RESIZED_HEIGHT) / 2);
	}

	bool IsInRecordMode() const
	{
		bool debugCameras = GetValue<bool>(RobotParameters::debugCameras);
		bool recordMode = GetValue<bool>(RobotParameters::recordMode);

		return debugCameras && recordMode && !RecordingStopped();
	}

	bool IsInReplayMode() const
	{
		bool debugCameras = GetValue<bool>(RobotParameters::debugCameras);
		bool replayMode = GetValue<bool>(RobotParameters::replayMode);

		return debugCameras && replayMode;
	}

	bool ShouldProcessVideo() const
	{
		bool startedProcessing = GetValue<bool>(RobotParameters::startedProcessing);
		return !IsInReplayMode() || startedProcessing;
	}

	bool IsInDebugMathMode() const
	{
		return GetValue<bool>(RobotParameters::debugMath);
	}

	void SetRecordMode(bool newRecordMode = true)
	{
		rip.SetValue(RobotParameters::recordMode, CastTo<std::string>(newRecordMode));
	}

	void SetReplayMode(bool newReplayMode = true)
	{
		rip.SetValue(RobotParameters::replayMode, CastTo<std::string>(newReplayMode));
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
		return currentImageIndex >= GetValue<int>(RobotParameters::numToRecord);
	}

	bool NoMoreImagesLeft() const
	{
		return RecordingStopped();
	}

	std::string BuildLeftImageName() const
	{
		return BuildImageName(GetValue<std::string>(RobotParameters::LeftSuffix));
	}

	std::string BuildRightImageName() const
	{
		return BuildImageName(GetValue<std::string>(RobotParameters::RightSuffix));
	}

	std::string BuildMatrixName(const std::string& wantedName) const
	{
		return GetValue<std::string>(RobotParameters::wrkDir) + 
			GetValue<std::string>(RobotParameters::Separator) +
			GetValue<std::string>(RobotParameters::OutputDirName) + 
			GetValue<std::string>(RobotParameters::Separator) +
			wantedName +
			PadWithZeroes(currentImageIndex, GetValue<int>(RobotParameters::MAX_ZERO_NUM)) + 
			GetValue<std::string>(RobotParameters::MatExtensionName);
	}

	std::string BuildCameraConfigFilename(int cameraId) const
	{
		return GetValue<std::string>(RobotParameters::wrkDir) +
			GetValue<std::string>(RobotParameters::Separator) +
			GetValue<std::string>(RobotParameters::CalibrationDirName) +
			GetValue<std::string>(RobotParameters::Separator) +
			GetValue<std::string>(RobotParameters::CameraCalibrationDataFileName) +
			CastTo<std::string>(cameraId) + 
			GetValue<std::string>(RobotParameters::CameraCalibrationDataFileExtension);
	}

	bool ShouldDebugCameras() const
	{
		return GetValue<bool>(RobotParameters::debugCameras);
	}

	void StartProcessing()
	{
		rip.SetValue(RobotParameters::startedProcessing, CastTo<std::string>(true));
	}

	void StopProcessing()
	{
		rip.SetValue(RobotParameters::startedProcessing, CastTo<std::string>(false));
	}

	void SetTurnSmoothness(bool smoothTurns)
	{
		rip.SetValue(RobotParameters::smoothTurn, CastTo<std::string>(smoothTurns));
	}
private:
	std::string BuildImageName(const std::string& suffix) const
	{
		return GetValue<std::string>(RobotParameters::wrkDir) +
			GetValue<std::string>(RobotParameters::Separator) +
			GetValue<std::string>(RobotParameters::ImagesDirName) +
			GetValue<std::string>(RobotParameters::Separator) +
			GetValue<std::string>(RobotParameters::BaseName) +
			suffix +
			PadWithZeroes(currentImageIndex, GetValue<int>(RobotParameters::MAX_ZERO_NUM)) +
			GetValue<std::string>(RobotParameters::ImageExtensionName);
	}
};


