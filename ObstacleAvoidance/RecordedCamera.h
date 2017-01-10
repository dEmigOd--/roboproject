#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RecordedCameraView class
//	enables the usage of stereo cameras on the systems that doesn't currently support this
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2017 Technion, IIT
//
//	2017, January 10
//
//M*/


#include <vector>

#include "Logger.h"
#include "CameraModel.h"

class RecordedCameraView : public CameraModel
{
private:
	static el::Logger* logger;

	int _id;

public:
	RecordedCameraView(const RunningParameters& params, int cameraId, const cv::Vec3d& cameraPosition = cv::Vec3d())
		: CameraModel(params, cameraId, cameraPosition, false), _id(cameraId)
	{
	}

	virtual cv::VideoCapture& operator >> (CV_OUT cv::Mat& image);

	virtual bool set(int propId, double value);

	virtual bool isOpened() const;
};