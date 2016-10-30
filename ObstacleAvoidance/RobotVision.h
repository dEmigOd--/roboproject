#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <memory>
#include <condition_variable>
#include <atomic>
#include "Logger.h"
#include "Common.h"
#include "CameraModel.h"
#include "Disparity.h"

class RobotVision
{
	static el::Logger* logger;

public:
	bool initialized;
	bool videoWorking;
	std::condition_variable condition;

private:
	std::unique_ptr<CameraModel> rightCap; // open the first camera
	std::unique_ptr<CameraModel> leftCap; // open the second camera
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
