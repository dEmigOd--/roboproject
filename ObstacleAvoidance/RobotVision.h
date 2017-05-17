#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotVision class
//	implements abstraction layer to extract knowledge of obstacle in the world
//
//
//	Author: ben, Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 17
//
//M*/


#include <vector>
#include <thread>
#include <mutex>
#include <memory>
#include <condition_variable>
#include <atomic>

#include "Logger.h"
#include "Common.h"
#include "RunningParameters.h"
#include "CameraModel.h"
#include "Disparity.h"

class RobotVision
{
	static el::Logger* logger;

public:
	bool initialized;
	bool videoWorking;
	std::condition_variable condition;
	std::shared_ptr<DepthCalculator> depthCalculator;

private:
	std::shared_ptr<CameraModel> rightCap; // open the first camera
	std::shared_ptr<CameraModel> leftCap; // open the second camera
	std::unique_ptr<Disparity> dispObj;
	cv::Mat left, right, leftRGB, rightRGB, leftForDisp, rightForDisp;

	cv::Mat disparity;
	double medianDisp;
	std::mutex mut;
	std::mutex& acqLock;
	std::atomic<bool> active;
	RunningParameters& params;
	std::thread videoGrab;

	void SafeCaptureFromCam();
	void SafeAcquireLastRecordedImages();
	bool VideoInWorkingState() const;

	void InitializeDisparityObject();
public:
	enum SideEnum
	{
		NONE,
		RIGHT,
		LEFT
	};

	RobotVision(std::mutex& acquisitionLock, RunningParameters& params);
	~RobotVision();
	void OpenVideoCap();
	cv::Mat& GetCurrentDisparity();
	cv::Mat& CalculateNewDisparity();
	RobotVision::SideEnum ObstaclePresent();
	cv::Mat GetMatches();
	void CaptureFromCam();
};
