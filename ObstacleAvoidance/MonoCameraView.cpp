#include "MonoCameraView.h"

el::Logger* MonoCameraView::logger = el::Loggers::getLogger("OneEyedVision");

cv::VideoCapture& MonoCameraView::operator >> (CV_OUT cv::Mat& image)
{
	VideoCapture underlying;
	underlying = VideoCapture(_id);

	if (!underlying.isOpened())  // check if we succeeded
	{
		logger->error("Camera id:%v initialization failed.", _id);
		CV_Error(CV_StsInternal, "At least one camera failed to initialize.");
	}

	// set stored properties
	for (auto const& ppty : onInitialize)
	{
		underlying.set(ppty.first, ppty.second);
	}
	
	// actually read image from camera
	underlying >> image;

	return *this;
}

bool MonoCameraView::set(int propId, double value)
{
	onInitialize.push_back(std::make_pair(propId, value));
	return true;
}

bool MonoCameraView::isOpened() const
{
	return true;
}
