/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	constants that were not initialized in .h
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include "Common.h"

const std::string Constants::LeftWindowName = "Left";
const std::string Constants::RightWindowName = "Right";
const std::string Constants::MatchesWindowName = "Matches";

const std::chrono::duration<long long, std::milli> Constants::CAPTURE_FROM_CAMERA_INTERVAL = std::chrono::milliseconds(10);
const double Constants::CLOSEST_DETECTABLE_RANGE = 0.4; //in seconds [that should be multiplied by speed to get distance]
const double Constants::FARTHEST_DETECTABLE_RANGE = 0.7; //in seconds [that should be multiplied by speed to get distance]
const double Constants::OPTICAL_AXIS_DISTANCE = 60.0; // mm
const double Constants::CHASSIS_WIDTH = 130.0; // mm
const double Constants::WHEEL_WIDTH = 20.0; // mm
const double Constants::OBSTACLE_LENGTH = 250.0; // mm
const double Constants::OBSTACLE_DISTANCE = 100.0; // mm

const std::string RunningParameters::BaseName = "RobotFov";
const std::string RunningParameters::ImageExtensionName = ".png";
const std::string RunningParameters::MatExtensionName = ".mat";
const std::string RunningParameters::Separator = "/";
const std::string RunningParameters::LeftSuffix = "_" + Constants::LeftWindowName + "_";
const std::string RunningParameters::RightSuffix = "_" + Constants::RightWindowName + "_";
const std::string RunningParameters::ImagesDirName = "images";
const std::string RunningParameters::OutputDirName = "output";
const std::string RunningParameters::CalibrationDirName = "calibration";
const std::string RunningParameters::CameraCalibrationDataFileName = "out_camera_data_";
const std::string RunningParameters::CameraCalibrationDataFileExtension = ".xml";
