#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Thresholds.h"
#include "Common.h"

using namespace std;

class Disparity
{
	RunningParameters& params;
	double CompareBlock(int i, int j, int k, const cv::Mat& right, const cv::Mat& left, const cv::Mat& edgeRight, const cv::Mat& edgeLeft) const;

	int ComputeBlockMatch(int i, int j, const cv::Mat& right, const cv::Mat& left, const cv::Mat& edgesRight,
		const cv::Mat& edgesLeft, const cv::Mat& rightBlockMap, const cv::Mat& leftBlockMap, const cv::Mat& disparity, 
		vector<vector<int>>& matchMap, int minDetectableDisparity, int maxDetectableDisparity) const;

public:
	Disparity(RunningParameters& params)
		: params(params)
	{
		if (!(params.nBlockSize % 2))
		{
			throw std::invalid_argument("Unsupported block size passed.");
		}
	}

	cv::Mat ComputeDisparityV2(const cv::Mat& left, const cv::Mat& right) const;
	cv::Mat ComputeDisparityV3(const cv::Mat& left, const cv::Mat& right) const;

	virtual ~Disparity()
	{
	}
};
