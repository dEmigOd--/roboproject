#pragma once

#include "OpenCv.h"
#include "Thresholds.h"
#include "Common.h"

class Disparity
{
protected:
	RunningParameters& params;

	Disparity(RunningParameters& params)
		: params(params)
	{
		if (!(params.nBlockSize % 2))
		{
			throw std::invalid_argument("Unsupported block size passed.");
		}
	}

public:
	virtual cv::Mat ComputeDisparity(const cv::Mat& left, const cv::Mat& right) const = 0;

};

class SGBMDisparity : public Disparity
{
public:
	SGBMDisparity(RunningParameters& params)
		: Disparity(params)
	{ }

	virtual cv::Mat ComputeDisparity(const cv::Mat& left, const cv::Mat& right) const;

};

class HomeGrownDisparity : public Disparity
{
public:
	HomeGrownDisparity(RunningParameters& params)
		: Disparity(params)
	{
	}

	virtual cv::Mat ComputeDisparity(const cv::Mat& left, const cv::Mat& right) const;

};
