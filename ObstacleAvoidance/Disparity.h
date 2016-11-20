#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	*	DepthCalculator class
//			implements calculation of depth from two images provided by two cameras
//			main drawback it needs a knowledge about the cameras on creation
//
//	*	RangeFinder class heirarchy
//			implements functions that should supply a meaningful disparities to check
//			supposes, cameras' resolutions do not change after startup
//			could adjust for speed changes on-the-fly
//
//	*	Disparity class hierarchy
//			implements calculating disparity from two images
//
//	Author(s): ben, Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include <memory>

#include "OpenCv.h"
#include "Common.h"
#include "CameraModel.h"

class DepthCalculator
{
	std::shared_ptr<CameraModel> leftCamera;
	std::shared_ptr<CameraModel> rightCamera;

public:
	DepthCalculator(std::shared_ptr<CameraModel> leftCamera, std::shared_ptr<CameraModel> rightCamera)
		: leftCamera(leftCamera), rightCamera(rightCamera)
	{ }

	cv::Mat GetDepthFromStereo(const cv::Mat& LeftImagePts, const cv::Mat& RightImagePts) const;
};

class RangeFinder
{
protected:
	RunningParameters& params;

public:
	RangeFinder(RunningParameters& params)
		: params(params)
	{}

	virtual int GetMinDisparity() const;
	virtual int GetMaxDisparity() const;
};
class DisparityRangeFinder : public RangeFinder
{
private:
	std::shared_ptr<DepthCalculator> depthCalculator;
	mutable int lastCalculatedSpeed = -1;
	mutable int minDisparity, maxDisparity;

	void DetectDisparities() const;
public:
	DisparityRangeFinder(RunningParameters& params, std::shared_ptr<DepthCalculator>& depthCalculator)
		: RangeFinder(params), depthCalculator(depthCalculator)
	{}

	virtual int GetMinDisparity() const;
	virtual int GetMaxDisparity() const;
};

class Disparity
{
protected:
	RunningParameters& params;
	std::unique_ptr<RangeFinder> rangeFinder;

	Disparity(RunningParameters& params)
		: params(params), rangeFinder(new RangeFinder(params))
	{
		if (!(params.nBlockSize % 2))
		{
			throw std::invalid_argument("Unsupported block size passed.");
		}
	}

public:
	virtual cv::Mat ComputeDisparity(const cv::Mat& left, const cv::Mat& right) const = 0;

	void SetRangeFinder(RangeFinder* newRangefinder)
	{
		rangeFinder = std::unique_ptr<RangeFinder>(newRangefinder);
	}
};

class SGBMDisparity : public Disparity
{
public:
	SGBMDisparity(RunningParameters& params)
		: Disparity(params)
	{ }

protected:
	virtual cv::Mat ComputeDisparity(const cv::Mat& left, const cv::Mat& right) const;
};

class HomeGrownDisparity : public Disparity
{
public:
	HomeGrownDisparity(RunningParameters& params)
		: Disparity(params)
	{
	}
protected:
	virtual cv::Mat ComputeDisparity(const cv::Mat& left, const cv::Mat& right) const;

};
