#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	DefaultParameters class
//	some default and constant values
//
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 11
//
//M*/


#include <map>
#include "RobotParameters.h"
#include "CastTo.h"

class DefaultParameters
{
	template<typename T>
	std::string MakeParam(const T& value) const
	{
		return CastTo<std::string>(value);
	}

public:
#define GetValue(value) params[(value)]
#define DivideInts(numerator, denominator) CastTo<int>(GetValue(numerator)) / CastTo<int>(GetValue(denominator))

	std::map<std::string, std::string> CreateDefaultParameters() const
	{
		std::map<std::string, std::string> params;

		params[RobotParameters::MAX_COLOR] = MakeParam(255); // 0-255 range

		params[RobotParameters::IMAGE_STRETCH_FACTOR] = MakeParam(1);
		params[RobotParameters::IMAGE_COMPRESS_FACTOR] = MakeParam(1);
		params[RobotParameters::CAMERA_FOV_WIDTH] = MakeParam(160); //pixels
		params[RobotParameters::CAMERA_FOV_HEIGHT] = MakeParam(120); //pixels
		params[RobotParameters::RESIZE_FACTOR] = MakeParam(1);

		params[RobotParameters::MatchesWindowName] = MakeParam("Matches");
		params[RobotParameters::LeftWindowName] = MakeParam("Left");
		params[RobotParameters::RightWindowName] = MakeParam("Right");

		params[RobotParameters::CAPTURE_FROM_CAMERA_INTERVAL] = MakeParam(10);

		params[RobotParameters::CLOSEST_DETECTABLE_RANGE] = MakeParam(0.4);//in seconds [that should be multiplied by speed to get distance]
		params[RobotParameters::FARTHEST_DETECTABLE_RANGE] = MakeParam(0.7);//in seconds [that should be multiplied by speed to get distance]
		params[RobotParameters::OPTICAL_AXIS_DISTANCE] = MakeParam(60.0); //mm
		params[RobotParameters::CHASSIS_WIDTH] = MakeParam(130.0); //mm
		params[RobotParameters::WHEEL_WIDTH] = MakeParam(20.0); //mm

		params[RobotParameters::OBSTACLE_LENGTH] = MakeParam(250.0); //mm
		params[RobotParameters::OBSTACLE_DISTANCE] = MakeParam(100.0); //mm


		params[RobotParameters::MAX_RECORD] = MakeParam(65536);
		params[RobotParameters::MAX_ZERO_NUM] = MakeParam(5);

		params[RobotParameters::ImagesDirName] = MakeParam("images");
		params[RobotParameters::OutputDirName] = MakeParam("output");
		params[RobotParameters::CalibrationDirName] = MakeParam("calibration");

		params[RobotParameters::CameraCalibrationDataFileName] = MakeParam("out_camera_data_");
		params[RobotParameters::CameraCalibrationDataFileExtension] = MakeParam(".xml");

		params[RobotParameters::BaseName] = MakeParam("RobotFov");
		params[RobotParameters::ImageExtensionName] = MakeParam(".png");
		params[RobotParameters::MatExtensionName] = MakeParam(".mat");
		params[RobotParameters::Separator] = MakeParam("/");

		const std::string UNDERSCORE = "_";

		params[RobotParameters::LeftSuffix] = MakeParam(UNDERSCORE + GetValue(RobotParameters::LeftWindowName) + UNDERSCORE);
		params[RobotParameters::RightSuffix] = MakeParam(UNDERSCORE + GetValue(RobotParameters::RightWindowName) + UNDERSCORE);

		params[RobotParameters::currentImageIndex] = MakeParam(0);
		params[RobotParameters::imagesReadPerIndex] = MakeParam(1);

		params[RobotParameters::recordMode] = MakeParam(false);
		params[RobotParameters::replayMode] = MakeParam(false);
		params[RobotParameters::startedProcessing] = MakeParam(false);
		params[RobotParameters::shouldMove] = MakeParam(true);
		params[RobotParameters::debugCameras] = MakeParam(false);
		params[RobotParameters::debugMath] = MakeParam(false);
		params[RobotParameters::numToRecord] = MakeParam(GetValue(RobotParameters::MAX_RECORD));

		params[RobotParameters::speed] = MakeParam(100);
		params[RobotParameters::speedUnitInMm] = MakeParam(7.0);

		params[RobotParameters::spinMultiplierOnTurns] = MakeParam(4.0);
		params[RobotParameters::AcceptableDepthOffset] = MakeParam(2);

		params[RobotParameters::leftCameraIdx] = MakeParam(1);
		params[RobotParameters::rightCameraIdx] = MakeParam(0);

		params[RobotParameters::useOpenCVAlgorihmForMatching] = MakeParam(OPENCV_SGBM);

		params[RobotParameters::heightOfHorizon] = MakeParam(100); //?
		params[RobotParameters::heightOfHorizonSet] = MakeParam(false);

		params[RobotParameters::frequency] = MakeParam(GetValue(RobotParameters::CAPTURE_FROM_CAMERA_INTERVAL));

		params[RobotParameters::LEFT_CAMERA_FOV_WIDTH] = MakeParam(GetValue(RobotParameters::CAMERA_FOV_WIDTH));
		params[RobotParameters::LEFT_CAMERA_FOV_HEIGHT] = MakeParam(GetValue(RobotParameters::CAMERA_FOV_HEIGHT));
		params[RobotParameters::RIGHT_CAMERA_FOV_WIDTH] = MakeParam(GetValue(RobotParameters::CAMERA_FOV_WIDTH));
		params[RobotParameters::RIGHT_CAMERA_FOV_HEIGHT] = MakeParam(GetValue(RobotParameters::CAMERA_FOV_HEIGHT));
		params[RobotParameters::LEFT_IMAGE_RESIZED_WIDTH] = MakeParam(DivideInts(RobotParameters::LEFT_CAMERA_FOV_WIDTH, RobotParameters::RESIZE_FACTOR));
		params[RobotParameters::LEFT_IMAGE_RESIZED_HEIGHT] = MakeParam(DivideInts(RobotParameters::LEFT_CAMERA_FOV_HEIGHT, RobotParameters::RESIZE_FACTOR));
		params[RobotParameters::RIGHT_IMAGE_RESIZED_WIDTH] = MakeParam(DivideInts(RobotParameters::RIGHT_CAMERA_FOV_WIDTH, RobotParameters::RESIZE_FACTOR));
		params[RobotParameters::RIGHT_IMAGE_RESIZED_HEIGHT] = MakeParam(DivideInts(RobotParameters::RIGHT_CAMERA_FOV_HEIGHT, RobotParameters::RESIZE_FACTOR));

		// pixels at them we'll pre-test disparities to have a clue, what are the ranges at given speed
		params[RobotParameters::NumWidthPixelsToTest] = MakeParam(40);
		params[RobotParameters::NumHeightPixelsToTest] = MakeParam(16);
		params[RobotParameters::DisparitiesToTestRatio] = MakeParam(2);

		params[RobotParameters::wrkDir] = MakeParam(".");
		params[RobotParameters::smoothTurn] = MakeParam(true);

		/////// SECTION INHERETED FROM PREVIOUS PROJECT ////////

		params[RobotParameters::nBlockSize] = MakeParam(9);
		//params[RobotParameters::THRESHOLD] = MakeParam(100);
		//params[RobotParameters::DENSITY] = MakeParam(100);
		//params[RobotParameters::OBST_THRESHOLD] = MakeParam(100);
		//params[RobotParameters::NPTS] = MakeParam(100);
		//params[RobotParameters::EVENT_THRESHOLD] = MakeParam(100);
		//params[RobotParameters::ERROR_THRESHOLD] = MakeParam(100);
		//params[RobotParameters::GAUSS_SIGMA] = MakeParam(100);
		//params[RobotParameters::LAPLACE_KERN] = MakeParam(100);
		//params[RobotParameters::DIST_THRESHOLD] = MakeParam(100);
		//params[RobotParameters::tresh] = MakeParam(100);

		return params;
	}

#undef DivideInts
#undef GetValue
};