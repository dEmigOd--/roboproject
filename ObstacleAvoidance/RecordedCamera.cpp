/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//  IMPORTANT: READ BEFORE USING.
//
//	MonoCameraView class
//	implements capturing images from stereo camera one camera at a time
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2017 Technion, IIT
//
//	2017, January 10
//
//M*/

#include "RecordedCamera.h"


el::Logger* RecordedCameraView::logger = el::Loggers::getLogger("PreRecordedVision");

cv::VideoCapture& RecordedCameraView::operator >> (CV_OUT cv::Mat& image)
{
	if (!params.NoMoreImagesLeft())
	{
		int imageIndexToRead = params.PrepareForNextImage();
		logger->trace("Reading image %v", PadWithZeroes(imageIndexToRead, 4));
	}

	image = cv::imread((_id == params.leftCameraIdx) ? 
		params.BuildLeftImageName() : 
		params.BuildRightImageName());

	return *this;
}

bool RecordedCameraView::set(int propId, double value)
{
	return true;
}

bool RecordedCameraView::isOpened() const
{
	return true;
}
