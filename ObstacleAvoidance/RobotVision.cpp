/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotVision class
//	implementation
//
//	Author: ben, Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include <algorithm>

#include "RobotVision.h"
#include "Utils.h"
#include "MonoCameraView.h"

const cv::Scalar RED(0, 0, 255); // left
const cv::Scalar GREEN(0, 255, 0); // no
const cv::Scalar BLUE(255, 0, 0); // right

el::Logger* RobotVision::logger = el::Loggers::getLogger("Vision");

RobotVision::RobotVision(std::mutex& acquisitionLock, RunningParameters& params) :
	initialized(false), acqLock(acquisitionLock), 
	active(true), params(params), videoGrab(&RobotVision::CaptureFromCam, this)
{
	InitializeDisparityObject();
	logger->debug("Inititalizing robot Vision System");
}

void RobotVision::InitializeDisparityObject()
{
	switch (params.useOpenCVAlgorihmForMatching)
	{
		case MatchingAlgorithm::OPENCV_SGBM:
			dispObj = std::unique_ptr<Disparity>(new SGBMDisparity(params));
			break;
		case MatchingAlgorithm::OWN_BLOCKMATCHING:
			dispObj = std::unique_ptr<Disparity>(new HomeGrownDisparity(params));
			break;
		default:
			throw std::invalid_argument("Unsupported block-matching algorithm");
	}
}

cv::Mat& RobotVision::GetCurrentDisparity()
{
	return disparity;
}

bool RobotVision::VideoInWorkingState() const
{
	return videoWorking;
}

class FrontPointCounter
{
	std::vector<std::vector<int>> closePoints;
	int maxDisparity = 0;
	int imageWidth;

public:

	FrontPointCounter(int imageWidth, int rangeWidth)
		: closePoints(2 * rangeWidth + 1), imageWidth(imageWidth)
	{
	}

	void AddPoint(int pointDisparity, int originalOffset)
	{
		// check out that the higher the disparity - closer the obstacle
		if (pointDisparity > maxDisparity)
		{
			for (int i = closePoints.size() - 1; i >= pointDisparity - maxDisparity; --i)
			{
				closePoints[i] = closePoints[i - (pointDisparity - maxDisparity)];
			}

			for (int i = 0; i < std::min(static_cast<int>(closePoints.size()), pointDisparity - maxDisparity); ++i)
			{
				closePoints[i] = std::vector<int>();
			}

			maxDisparity = pointDisparity;
		}

		if (maxDisparity - pointDisparity < static_cast<int>(closePoints.size()))
		{
			closePoints[maxDisparity - pointDisparity].push_back(originalOffset);
		}		
	}

	RobotVision::SideEnum DetectSide() const
	{
		int leftCount(0), rightCount(0);

		for (int list = 0; list < static_cast<int>(closePoints.size()); ++list)
		{
			for (auto const& offset : closePoints[list])
			{
				// remember max - disparity is distance between optical axis, so center is half that distance to the right of left image center
				// additionally column numbering increases from left to right, hence the "+" sign
				if (offset < (imageWidth + (maxDisparity - list)) / 2)
				{
					++leftCount;
				}
				else // even in the center
				{
					++rightCount;
				}
			}
		}

		return leftCount > rightCount ? RobotVision::LEFT :
			leftCount < rightCount ? RobotVision::RIGHT : RobotVision::NONE;
	}

	int NumMatchedPoints() const
	{
		int matchedPoints = 0;
		for (auto const& list : closePoints)
		{
			matchedPoints += list.size();
		}

		return matchedPoints;
	}

};

RobotVision::SideEnum RobotVision::ObstaclePresent()
{
	RobotVision::SideEnum res = NONE;

	if (!VideoInWorkingState())
	{
		return res;
	}

	cv::Mat disparity = CalculateNewDisparity();
	FrontPointCounter pc(disparity.cols, params.AcceptableDepthOffset);

	for (int i = 0; i < disparity.rows; i++)
	{
		for (int j = 0; j < disparity.cols; j++)
		{
			// count only close to front points !!
			pc.AddPoint((int) disparity.at<short>(i, j), j);
		}
	}

	cv::Mat obst(disparity.rows, 10, CV_8UC3);
	obst = GREEN;

	if (pc.NumMatchedPoints() >= params.NPTS)
	{
		res = pc.DetectSide();
		logger->verbose(2, "Detected obstacle from %v", res);

		if (res == RobotVision::LEFT)
		{
			obst = RED;
		}
		else // otherwise from the right
		{
			obst = BLUE;
		}

	}
	cv::Mat matches = GetMatches();

	cv::Mat resMat;
	cv::hconcat(matches, obst, resMat);
	cv::imshow("Matches", resMat);

	return res;
}

void RobotVision::SafeAcquireLastRecordedImages()
{
	std::lock_guard<std::mutex> lock(mut);

	leftForDisp = left;
	rightForDisp = right;

	if (params.IsInRecordMode())
	{
		int imageIndexToWrite = params.PrepareForNextImage();

		logger->trace("Image %v written", PadWithZeroes(imageIndexToWrite, 4));

		cv::imwrite(params.BuildLeftImageName(), leftForDisp);
		cv::imwrite(params.BuildRightImageName(), rightForDisp);
	}
}

cv::Mat& RobotVision::CalculateNewDisparity()
{
	SafeAcquireLastRecordedImages();

	cv::Mat leftImage, rightImage;
	// undistort the images the images
	cv::undistort(leftForDisp, leftImage, leftCap->K(), leftCap->Distortion());
	cv::undistort(rightForDisp, rightImage, rightCap->K(), rightCap->Distortion());

	this->disparity = dispObj->ComputeDisparity(leftImage, rightImage);

	return this->disparity;
}

cv::Mat RobotVision::GetMatches()
{
	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	std::vector<cv::DMatch> matches;

	int k = 0;
	cv::Mat currDisparity = GetCurrentDisparity();

	int step = std::max(1, params.DENSITY);

	for (int y = 0; y < left.rows; y += step)
	{
		for (int x = 0; x < left.cols; x += step)
		{
			// check out http://stackoverflow.com/questions/25642532/opencv-pointx-y-represent-column-row-or-row-column
			int disparity = (int)currDisparity.at<short>(cv::Point(x, y));
			if (disparity > params.OBST_THRESHOLD)
			{
				cv::KeyPoint point(x, y, 1);
				keypoints_1.push_back(point);
				cv::KeyPoint point2(x - disparity, y, 1);
				keypoints_2.push_back(point2);
				cv::DMatch match(k, k, 0);
				matches.push_back(match);
				k++;
			}
		}
	}

	cv::Mat img_matches;

	cv::drawMatches(leftForDisp, keypoints_1, rightForDisp, keypoints_2, matches, img_matches);
	return img_matches;
}

void RobotVision::OpenVideoCap()
{
	logger->debug("Initializing cameras ...");

	if (!params.IsInReplayMode())
	{
		cv::Vec3d leftCameraPosition = cv::Vec3d({ -Constants::OPTICAL_AXIS_DISTANCE / 2, 0.0, 0.0 });
		cv::Vec3d rightCameraPosition = cv::Vec3d({ Constants::OPTICAL_AXIS_DISTANCE / 2, 0.0, 0.0 });

#ifdef __linux__
		leftCap = std::unique_ptr<CameraModel>(new CameraModel(params, params.leftCameraIdx, leftCameraPosition));
		rightCap = std::unique_ptr<CameraModel>(new CameraModel(params, params.rightCameraIdx, rightCameraPosition));
#else
		leftCap = std::unique_ptr<CameraModel>(new MonoCameraView(params, params.leftCameraIdx, leftCameraPosition));
		rightCap = std::unique_ptr<CameraModel>(new MonoCameraView(params, params.rightCameraIdx, rightCameraPosition));
#endif

		if (!rightCap->isOpened() || !leftCap->isOpened())  // check if we succeeded
		{
			logger->error("Camera(s) initialization failed. Left opened=%v, Right opened=%v", rightCap->isOpened(), leftCap->isOpened());
			CV_Error(CV_StsInternal, "At least one camera failed to initialize.");
		}

		// some wonderful properties of ld, which was unable to resolve those static const ints to ints
		logger->info("Setting right camera FOV to %v-by-%v", (int)params.RIGHT_CAMERA_FOV_WIDTH, (int)params.RIGHT_CAMERA_FOV_HEIGHT);
		rightCap->set(CV_CAP_PROP_FRAME_WIDTH, params.RIGHT_CAMERA_FOV_WIDTH);
		rightCap->set(CV_CAP_PROP_FRAME_HEIGHT, params.RIGHT_CAMERA_FOV_HEIGHT);

		logger->info("Setting left camera FOV to %v-by-%v", (int)params.LEFT_CAMERA_FOV_WIDTH, (int)params.LEFT_CAMERA_FOV_HEIGHT);
		leftCap->set(CV_CAP_PROP_FRAME_WIDTH, params.LEFT_CAMERA_FOV_WIDTH);
		leftCap->set(CV_CAP_PROP_FRAME_HEIGHT, params.LEFT_CAMERA_FOV_HEIGHT);

	}

	depthCalculator = std::shared_ptr<DepthCalculator>(new DepthCalculator(leftCap, rightCap));
	dispObj->SetRangeFinder(new DisparityRangeFinder(params, depthCalculator));

	{
		// signal initialization
		std::lock_guard<std::mutex> acquisitionLock(acqLock);

		initialized = true;
	}

	logger->debug("Cameras successfully initialized.");
	condition.notify_one();
}

void RobotVision::SafeCaptureFromCam()
{
	std::lock_guard<std::mutex> lock(mut);

	if (params.IsInReplayMode())
	{
		if (!params.ShouldProcessVideo())
		{
			return;
		}

		if (!params.NoMoreImagesLeft())
		{
			int imageIndexToRead = params.PrepareForNextImage();
			logger->trace("Reading image %v", PadWithZeroes(imageIndexToRead, 4));
		}

		left = cv::imread(params.BuildLeftImageName());
		right = cv::imread(params.BuildRightImageName());
	}
	else
	{
		*leftCap >> left;
		*rightCap >> right;
	}

	videoWorking = true;

	resize(left, left, cv::Size(params.LEFT_IMAGE_RESIZED_WIDTH, params.LEFT_IMAGE_RESIZED_HEIGHT));
	resize(right, right, cv::Size(params.RIGHT_IMAGE_RESIZED_WIDTH, params.RIGHT_IMAGE_RESIZED_HEIGHT));

	cvtColor(left, left, CV_BGR2GRAY);
	cvtColor(right, right, CV_BGR2GRAY);
}

void RobotVision::CaptureFromCam()
{
	OpenVideoCap();

	while (active)
	{
		SafeCaptureFromCam();

		std::this_thread::sleep_for(params.frequency);
	}
}

RobotVision :: ~RobotVision()
{
	active = false;
	// not sure why i need a lock, may be in case at start
	{
		std::lock_guard<std::mutex> lock(mut);
	}

	videoGrab.join();
	logger->debug("Closing robot Vision System");
}
