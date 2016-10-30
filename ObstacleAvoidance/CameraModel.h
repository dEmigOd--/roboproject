#pragma once

#include "OpenCv.h"
#include "CameraConfig.h"

class CameraModel : public cv::VideoCapture, public CalibratedConfig
{
	bool fallThrough;
public:
	CameraModel(int cameraId, const cv::Vec3d& cameraPosition = cv::Vec3d(), bool fallThrough = true)
		: VideoCapture(cameraId), CalibratedConfig(cameraPosition), fallThrough(fallThrough)
	{
	}

	virtual bool set(int propId, double value);
};