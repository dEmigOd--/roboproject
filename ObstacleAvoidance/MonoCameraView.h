#pragma once
#include <vector>
#include "CameraModel.h"
#include "Logger.h"

class MonoCameraView : public CameraModel
{
private:
	static el::Logger* logger;

	int _id;
	std::vector<std::pair<int, double>> onInitialize;

public:
	MonoCameraView(int cameraId, const cv::Vec3d& cameraPosition = cv::Vec3d())
		: CameraModel(cameraId, cameraPosition, false), _id(cameraId), onInitialize()
	{
	}

	virtual cv::VideoCapture& operator >> (CV_OUT cv::Mat& image);

	virtual bool set(int propId, double value);

	virtual bool isOpened() const;
};