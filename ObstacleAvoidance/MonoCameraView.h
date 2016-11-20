#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	MonoCameraView class
//	enables the usage of stereo cameras on the systems that doesn't currently support this
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/


#include <vector>

#include "Logger.h"
#include "CameraModel.h"

class MonoCameraView : public CameraModel
{
private:
	static el::Logger* logger;

	int _id;
	std::vector<std::pair<int, double>> onInitialize;

public:
	MonoCameraView(const RunningParameters& params, int cameraId, const cv::Vec3d& cameraPosition = cv::Vec3d())
		: CameraModel(params, cameraId, cameraPosition, false), _id(cameraId), onInitialize()
	{
	}

	virtual cv::VideoCapture& operator >> (CV_OUT cv::Mat& image);

	virtual bool set(int propId, double value);

	virtual bool isOpened() const;
};