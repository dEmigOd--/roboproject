#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotParameters class
//	stores input parameter strings
//
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 11
//
//M*/


#define PI (3.141592653589793238463)

// Check out that bool command parameter should be passed as 0/1
struct RobotParameters
{
public:

	static constexpr const char* MAX_COLOR = "MAX_COLOR";

	static constexpr const char* IMAGE_STRETCH_FACTOR = "IMAGE_STRETCH_FACTOR";
	static constexpr const char* IMAGE_COMPRESS_FACTOR = "IMAGE_COMPRESS_FACTOR";
	static constexpr const char* CAMERA_FOV_WIDTH = "CAMERA_FOV_WIDTH";
	static constexpr const char* CAMERA_FOV_HEIGHT = "CAMERA_FOV_HEIGHT";
	static constexpr const char* RESIZE_FACTOR = "RESIZE_FACTOR";

	static constexpr const char* MatchesWindowName = "MatchesWindowName";
	static constexpr const char* LeftWindowName = "LeftWindowName";
	static constexpr const char* RightWindowName = "RightWindowName";
	static constexpr const char* CAPTURE_FROM_CAMERA_INTERVAL = "CAPTURE_FROM_CAMERA_INTERVAL";
	static constexpr const char* CLOSEST_DETECTABLE_RANGE = "CLOSEST_DETECTABLE_RANGE";
	static constexpr const char* FARTHEST_DETECTABLE_RANGE = "FARTHEST_DETECTABLE_RANGE";
	static constexpr const char* OPTICAL_AXIS_DISTANCE = "OPTICAL_AXIS_DISTANCE"; // mm
	static constexpr const char* CHASSIS_WIDTH = "CHASSIS_WIDTH"; // mm
	static constexpr const char* WHEEL_WIDTH = "WHEEL_WIDTH"; // mm

	static constexpr const char* OBSTACLE_LENGTH = "OBSTACLE_LENGTH"; // mm
	static constexpr const char* OBSTACLE_DISTANCE = "OBSTACLE_DISTANCE"; // mm


	static constexpr const char* MAX_RECORD = "MAX_RECORD";
	static constexpr const char* MAX_ZERO_NUM = "MAX_ZERO_NUM";

	static constexpr const char* ImagesDirName = "ImagesDirName";
	static constexpr const char* OutputDirName = "OutputDirName";
	static constexpr const char* CalibrationDirName = "CalibrationDirName";

	static constexpr const char* CameraCalibrationDataFileName = "CameraCalibrationDataFileName";
	static constexpr const char* CameraCalibrationDataFileExtension = "CameraCalibrationDataFileExtension";

	static constexpr const char* BaseName = "BaseName";
	static constexpr const char* ImageExtensionName = "ImageExtensionName";
	static constexpr const char* MatExtensionName = "MatExtensionName";
	static constexpr const char* Separator = "Separator";
	static constexpr const char* LeftSuffix = "LeftSuffix";
	static constexpr const char* RightSuffix = "RightSuffix";

	static constexpr const char* currentImageIndex = "currentImageIndex";
	static constexpr const char* imagesReadPerIndex = "imagesReadPerIndex";

	static constexpr const char* speed = "speed";
	static constexpr const char* speedUnitInMm = "speedUnitInMm";

	static constexpr const char* spinMultiplierOnTurns = "spinMultiplierOnTurns";
	static constexpr const char* AcceptableDepthOffset = "AcceptableDepthOffset";

	static constexpr const char* leftCameraIdx = "leftCameraIdx";
	static constexpr const char* rightCameraIdx = "rightCameraIdx";

	static constexpr const char* useOpenCVAlgorihmForMatching = "useOpenCVAlgorihmForMatching";

	static constexpr const char* frequency = "frequency";

	static constexpr const char* LEFT_CAMERA_FOV_WIDTH = "LEFT_CAMERA_FOV_WIDTH";
	static constexpr const char* LEFT_CAMERA_FOV_HEIGHT = "LEFT_CAMERA_FOV_HEIGHT";
	static constexpr const char* RIGHT_CAMERA_FOV_WIDTH = "RIGHT_CAMERA_FOV_WIDTH";
	static constexpr const char* RIGHT_CAMERA_FOV_HEIGHT = "RIGHT_CAMERA_FOV_HEIGHT";
	static constexpr const char* LEFT_IMAGE_RESIZED_WIDTH = "LEFT_IMAGE_RESIZED_WIDTH";
	static constexpr const char* LEFT_IMAGE_RESIZED_HEIGHT = "LEFT_IMAGE_RESIZED_HEIGHT";
	static constexpr const char* RIGHT_IMAGE_RESIZED_WIDTH = "RIGHT_IMAGE_RESIZED_WIDTH";
	static constexpr const char* RIGHT_IMAGE_RESIZED_HEIGHT = "RIGHT_IMAGE_RESIZED_HEIGHT";

	// pixels at them we'll pre-test disparities to have a clue, what are the ranges at given speed
	static constexpr const char* NumWidthPixelsToTest = "NumWidthPixelsToTest";
	static constexpr const char* NumHeightPixelsToTest = "NumHeightPixelsToTest";
	static constexpr const char* DisparitiesToTestRatio = "DisparitiesToTestRatio";

	static constexpr const char* wrkDir = "wrkDir";
	static constexpr const char* smoothTurn = "smoothTurn";

	/////// SECTION INHERETED FROM PREVIOUS PROJECT ////////

	static constexpr const char* nBlockSize = "nBlockSize";
	static constexpr const char* THRESHOLD = "THRESHOLD";
	static constexpr const char* DENSITY = "DENSITY";
	static constexpr const char* OBST_THRESHOLD = "OBST_THRESHOLD";
	static constexpr const char* NPTS = "NPTS";
	static constexpr const char* EVENT_THRESHOLD = "EVENT_THRESHOLD";
	static constexpr const char* ERROR_THRESHOLD = "ERROR_THRESHOLD";
	static constexpr const char* GAUSS_SIGMA = "GAUSS_SIGMA";
	static constexpr const char* LAPLACE_KERN = "LAPLACE_KERN";
	static constexpr const char* DIST_THRESHOLD = "DIST_THRESHOLD";
	static constexpr const char* tresh = "tresh";


	/////// DEBUG SECTION ////////

	static constexpr const char* heightOfHorizon = "heightOfHorizon";
	static constexpr const char* recordMode = "recordMode";
	static constexpr const char* replayMode = "replayMode";
	static constexpr const char* startedProcessing = "startedProcessing";
	static constexpr const char* shouldMove = "shouldMove";
	static constexpr const char* debugCameras = "debugCameras";
	static constexpr const char* debugMath = "debugMath";
	static constexpr const char* numToRecord = "numToRecord";

	static constexpr const char* heightOfHorizonSet = "heightOfHorizonSet";

};
