/*
 * Disparity.cpp
 *
 *  Created on: May 16, 2015
 *      Author: ben
 *  Reworked on: September 6, 2016
 *      Author: dEmigOd
 */

#include "Disparity.h"
#include "Common.h"
#include "Utils.h"
#include <limits>
#include <fstream>
#include <math.h>

cv::Mat DetectEdges(const cv::Mat& image)
{
	// allocate grayscale matrix of same size as given image
	cv::Mat edges = cv::Mat::zeros(image.size(), CV_8U);

	// smooth with Gaussian
	cv::GaussianBlur(image, edges, cv::Size(GAUSS_SIGMA, GAUSS_SIGMA), 0, 0, cv::BORDER_DEFAULT);
	// apply Laplacian kernel to detect edges
	cv::Laplacian(edges, edges, CV_8U, LAPLACE_KERN, 1, 0, cv::BORDER_DEFAULT);

	return edges;
}

cv::Mat ThresholdEdges(cv::Mat& edges)
{
	cv::Mat edgesThresh = cv::Mat::zeros(edges.size(), CV_8U);
	// i don't see why this is useful - may be making edges broader
	cv::medianBlur(edges, edges, 3);

	// make edges much more visible
	cv::threshold(edges, edgesThresh, THRESHOLD, Constants::MAX_COLOR, cv::THRESH_BINARY);

	return edgesThresh;
}

void CalculateDetectableRange(const RunningParameters& params, int& minDetectableDisparity, int& maxDetectableDisparity)
{
	// we are mostly interested in decreasing greatly the search space
	// so we'll only look for matches in 1/3-1/2 seconds away from us
	double farthest_image_width = 2 * (params.speed * params.speedUnitInMm) * Constants::FARTHEST_DETECTABLE_RANGE * Constants::CAMERA_ANGLE_WIDTH_MULTIPLIER;
	double closest_image_width = 2 * (params.speed * params.speedUnitInMm) * Constants::CLOSEST_DETECTABLE_RANGE * Constants::CAMERA_ANGLE_WIDTH_MULTIPLIER;

	// in left image only up-to cameras' axis distance detectable should be checked
	minDetectableDisparity = std::min((int)(Constants::OPTICAL_AXIS_DISTANCE / farthest_image_width * params.LEFT_IMAGE_RESIZED_WIDTH), params.LEFT_IMAGE_RESIZED_WIDTH);
	maxDetectableDisparity = std::min((int)(Constants::OPTICAL_AXIS_DISTANCE / closest_image_width * params.LEFT_IMAGE_RESIZED_WIDTH) + 1, params.LEFT_IMAGE_RESIZED_WIDTH);
}

void AdjustDetectableRange(const RunningParameters& params, int& minDetectableDisparity, int& maxDetectableDisparity)
{
	int constDisparityDivider = 16;
	int constBitMask = constDisparityDivider - 1; // 0b1111

	// opencv demands num of detectable disparities to be a multiple of 16
	int missingTo16 = (int)(unsigned)((minDetectableDisparity - maxDetectableDisparity) & constBitMask);
	int couldBeAdded = params.LEFT_IMAGE_RESIZED_WIDTH + minDetectableDisparity - maxDetectableDisparity;

	// adjust symmetrically in case we can't add enough pixels from both sides
	if (missingTo16 > couldBeAdded)
	{
		minDetectableDisparity += ((constDisparityDivider - missingTo16) / 2);
		maxDetectableDisparity -= ((constDisparityDivider - missingTo16) / 2);

		return;
	}

	// add missing pixels, since now there is enough room for the missing
	int maxToAddOnLeft = minDetectableDisparity;
	int maxToAddOnRight = params.LEFT_IMAGE_RESIZED_WIDTH - maxDetectableDisparity;

	if (missingTo16 % 2)
	{
		if (maxToAddOnLeft)
		{
			--minDetectableDisparity;
			--maxToAddOnLeft;
			--missingTo16;
		}
	}

	int minToAdd = std::min(std::min(maxToAddOnLeft, missingTo16 / 2), std::min(maxToAddOnRight, missingTo16 / 2));

	// add on both sides
	minDetectableDisparity -= minToAdd;
	maxDetectableDisparity += minToAdd;
	missingTo16 -= 2 * minToAdd;

	// what's left add there is only possible
	if (minDetectableDisparity)
	{
		minDetectableDisparity -= missingTo16;
	}
	else
	{
		maxDetectableDisparity += missingTo16;
	}
}

void ExtractSignificantEdges(const cv::Mat& left, const cv::Mat& right, cv::Mat& edgesLeftThresh, cv::Mat& edgesRightThresh)
{
	// do it on edges, not original images
	cv::Mat edgesLeft = DetectEdges(left);
	cv::Mat edgesRight = DetectEdges(right);

	edgesLeftThresh = ThresholdEdges(edgesLeft);
	edgesRightThresh = ThresholdEdges(edgesRight);

	cv::imshow(Constants::LeftWindowName, edgesRightThresh);
	cv::imshow(Constants::RightWindowName, edgesLeftThresh);
	cv::waitKey(1);
}

cv::Ptr<cv::StereoSGBM> CreateSGBM(const RunningParameters& params, int channels, int minDetectableDisparity, int numDisparities)
{
	int uniquenessRatio = 10;
	int speckleWindowSize = 100;
	int speckleRange = 32;
	int disp12MaxDiff = 1;

	int sgbmWinSize = params.nBlockSize;

	cv::Ptr<cv::StereoSGBM> sgbm =
#if CV_VERSION_MAJOR < 3
		new cv::StereoSGBM(minDetectableDisparity, numDisparities, sgbmWinSize,
			uniquenessRatio, speckleWindowSize, speckleRange, disp12MaxDiff,
			8 * 3 * sgbmWinSize * sgbmWinSize,
			32 * 3 * sgbmWinSize * sgbmWinSize,
			false);
#else
	cv::StereoSGBM::create(minDetectableDisparity, numDisparities, sgbmWinSize);
	sgbm->setPreFilterCap(63);
	sgbm->setBlockSize(sgbmWinSize);

	sgbm->setP1(8 * channels * sgbmWinSize * sgbmWinSize);
	sgbm->setP2(32 * channels * sgbmWinSize * sgbmWinSize);
	sgbm->setMinDisparity(minDetectableDisparity);
	sgbm->setNumDisparities(numDisparities);
	sgbm->setUniquenessRatio(uniquenessRatio);
	sgbm->setSpeckleWindowSize(speckleWindowSize);
	sgbm->setSpeckleRange(speckleRange);
	sgbm->setDisp12MaxDiff(disp12MaxDiff);

#endif
	return sgbm;
}

cv::Mat Disparity::ComputeDisparityV3(const cv::Mat& left, const cv::Mat& right) const
{
	// set possible hit region
	int minDetectableDisparity, maxDetectableDisparity;

	// make a divisable by 16 number of disparities
	CalculateDetectableRange(params, minDetectableDisparity, maxDetectableDisparity);
	AdjustDetectableRange(params, minDetectableDisparity, maxDetectableDisparity);

	// make a divisable by 16 number of disparities
	int numDisparities = maxDetectableDisparity - minDetectableDisparity;

	// minDetectableDisparity = 0;
	cv::Ptr<cv::StereoSGBM> sgbm = CreateSGBM(params, left.channels(), minDetectableDisparity, numDisparities);

	cv::Mat edgesLeftThresh;
	cv::Mat edgesRightThresh;

	ExtractSignificantEdges(left, right, edgesLeftThresh, edgesRightThresh);

	cv::Mat disparity, disparity16S, maskedDisparity;
	// compute disparities
#if CV_VERSION_MAJOR < 3
	sgbm->operator()
#else
	sgbm->compute
#endif
	(right, left, disparity);

	// convert to human-acceptable visually
	disparity.convertTo(disparity16S, CV_16S, 1 / 16.);
	// now we are only interested in edges, so mask out non-edge stuff
	disparity16S.copyTo(maskedDisparity, (disparity16S >= minDetectableDisparity) & (edgesRightThresh >= THRESHOLD));

	if (params.IsInDebugMathMode())
	{
		WriteCSV(params.BuildMatrixName("Disparity"), disparity);
		WriteCSV(params.BuildMatrixName("Disparity16S"), disparity16S);
		WriteCSV(params.BuildMatrixName("MaskedDisparity"), maskedDisparity);
	}

	// zero all pixels under horizon
	maskedDisparity
		.rowRange(params.GetHorizonHeight() + 1, maskedDisparity.rows)
		.setTo(cv::Scalar(0));

	return maskedDisparity;
}

cv::Mat Disparity::ComputeDisparityV2(const cv::Mat& left, const cv::Mat& right) const
{
	cv::Mat edgesLeft = DetectEdges(left);
	cv::Mat edgesRight = DetectEdges(right);

	cv::Mat edgesLeftThresh = ThresholdEdges(edgesLeft);
	cv::Mat edgesRightThresh = ThresholdEdges(edgesRight);

	cv::imshow(Constants::LeftWindowName, edgesRightThresh);
	cv::imshow(Constants::RightWindowName, edgesLeftThresh);
	cv::waitKey(1);

	cv::Mat disparity = cv::Mat::zeros(edgesLeft.size(), CV_16S);

	vector<vector<int>> matchMap(disparity.rows, vector<int>(disparity.cols, 0));

	// set possible hit region
	int minDetectableDisparity, maxDetectableDisparity;
	CalculateDetectableRange(params, minDetectableDisparity, maxDetectableDisparity);


	// only down to horizon to suppress matching ground points
	for (int i = 0; i < params.GetHorizonHeight(); ++i)
	{
		// proceed with a horizontal scan
		for (int j = minDetectableDisparity; j < disparity.cols; ++j)
		{
            int disp = ComputeBlockMatch(i, j, right, left, edgesRight, edgesLeft, edgesRightThresh, edgesLeftThresh, disparity, matchMap, 
				minDetectableDisparity, maxDetectableDisparity);
			// if found matching block store its disparity off mine image
			// disparity is positive - we preserve j - k difference between right image coordinate and left
			disparity.at<short>(i, j) = disp;

			// match found and not yet stored [i.e.
			if (disp != 0 && matchMap[i][j - disp] == 0)
			{
				matchMap[i][j - disp] = j;
			}
		}
	}

	if (params.IsInDebugMathMode())
	{
		WriteCSV("output/Disparity.mat", disparity);
	}

	cv::medianBlur(disparity, disparity, MEDIAN_BLUR_WINDOW);

	return disparity;
}

int Disparity::ComputeBlockMatch(int i, int j, const cv::Mat& left, const cv::Mat& right, const cv::Mat& edgesLeft, const cv::Mat& edgesRight,
	const cv::Mat& leftBlockMap, const cv::Mat& rightBlockMap, const cv::Mat& disparity,
	vector<vector<int>>& matchMap, int minDetectableDisparity, int maxDetectableDisparity) const
{
	// if block is not on the edge leave
	if (rightBlockMap.at<uchar>(i, j) == 0)
	{
		return 0;
	}

	//int col = (j == 0) ? j : j - 1;
	//int row = (i == 0) ? i : i - 1;

	vector<pair<double, int>> matches;
	double min = numeric_limits<double>::max();
	pair<double, int> firstErr, secondErr, thirdErr;
	int disp = 0;
	double leftErr(0), upErr(0), smoothnessErr(0), edgeProfileError(0);
	double err(0), blockErr(0);

	// in the closest range we could save few more cycles in not scanning till the end
	int stopK = j - minDetectableDisparity + OBST_THRESHOLD;
	int startK = std::max(0, j - maxDetectableDisparity - OBST_THRESHOLD);

	// ignore objects in infinity or too far away, same j !
	for (int k = stopK - 1; k >= startK; --k)
	{
		if (leftBlockMap.at<uchar>(i, k) != 0)
		{
			// we want to count for found disparity above/left etc us with similarity in shifted pixels
			//upErr = (int)disparity.at<short>(row, j) != 0 ? abs((k - j) - (int)disparity.at<short>(row, j)) : 0;
			//upErr = upErr / right.rows + abs((int)right.at<uchar>(row, j) - (int)right.at<uchar>(i, j)) / Constants::MAX_COLOR;

			//leftErr = (int)disparity.at<short>(i, col) != 0 ? abs((k - j) - (int)disparity.at<short>(i, col)) : 0;
			//leftErr = leftErr / right.cols + abs((int)right.at<uchar>(i, col) - (int)right.at<uchar>(i, j)) / Constants::MAX_COLOR;

			blockErr = CompareBlock(i, j, k, right, left, edgesRight, edgesLeft);
			smoothnessErr = (leftErr + upErr) == 0 ? 1 : (leftErr + upErr);

			err = smoothnessErr * blockErr * edgeProfileError;

			if (abs(matchMap[i][k] - j) < 3 || matchMap[i][k] == 0)
			{

				matches.push_back(make_pair(err, j - k));
				if (err < min)
				{
					min = err;
					disp = j - k;
				}
			}
		}
	}
	sort(matches.begin(), matches.end());
	vector<pair<double, int>>::iterator it = matches.begin();


	if (it == matches.end())
	{
		return 0;
	}

	firstErr = *it++;
	if (it == matches.end())
	{
		return disp;
	}

	secondErr = *it++;


	if ((secondErr.first - firstErr.first) / secondErr.first < ERROR_THRESHOLD)
	{
		// some kind of pattern detection
		if (abs(secondErr.second - firstErr.second) <= DIST_THRESHOLD)
		{
			if (it == matches.end())
			{
				return disp;
			}

			thirdErr = *it;
			if ((thirdErr.first - firstErr.first) / thirdErr.first < ERROR_THRESHOLD)
			{
				return 0;
			}
		}
		else
		{
			return 0;
		}
	}


	return  disp;
}

double Disparity::CompareBlock(int i, int j, int k, const cv::Mat &left, const cv::Mat &right, const cv::Mat &edgeLeft, const cv::Mat &edgeRight) const
{
	// ensures proper index access - no off image
	int halfBlockSize = (params.nBlockSize >> 1) << 1;

	int blockWidthLeft = min(halfBlockSize, min(j, k));
	int blockWidthRight = min(halfBlockSize, min(right.cols - j - 1, right.cols - k - 1));

	int blockHeightTop = min(halfBlockSize, i);
	int blockHeightBottom = min(halfBlockSize, right.rows - i - 1);

	double sad = 0;
	double normalizer = 0;

	// per paper this is the interesting Score result
	for (int row = i - blockHeightTop; row <= i + blockHeightBottom; ++row)
	{
		for (int col = j - blockWidthLeft; col <= j + blockWidthRight; ++col)
		{
			double leftI = left.at<uchar>(row, col - j + k);
			double rightI = right.at<uchar>(row, col);
			double leftEdge = edgeLeft.at<uchar>(row, col - j + k);
			double rightEdge = edgeRight.at<uchar>(row, col);

			sad += abs(rightI - leftI);
			normalizer += rightEdge + leftEdge;
		}
	}

	return (normalizer) ? sad / normalizer : sad;
}

