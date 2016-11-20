#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	CameraModel class
//	abstracts the camera. Ties together low-level camera and its physical properties
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/


#include "OpenCv.h"
#include "CameraConfig.h"

class CameraModel : public cv::VideoCapture, public CalibratedConfig
{
	bool fallThrough;
public:
	CameraModel(const RunningParameters& params, int cameraId, const cv::Vec3d& cameraPosition = cv::Vec3d(), bool fallThrough = true)
		: VideoCapture(cameraId), CalibratedConfig(params, cameraId, cameraPosition), fallThrough(fallThrough)
	{
	}

	virtual bool set(int propId, double value);
};
