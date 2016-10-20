#pragma once
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "Logger.h"

class MonoCameraView : public cv::VideoCapture
{
private:
	static el::Logger* logger;

	int _id;
	std::vector<std::pair<int, double>> onInitialize;

public:
	MonoCameraView(int cameraId)
		: _id(cameraId), onInitialize()
	{
	}

	virtual cv::VideoCapture& operator >> (CV_OUT cv::Mat& image);

	virtual bool set(int propId, double value);

	virtual bool isOpened() const;
};