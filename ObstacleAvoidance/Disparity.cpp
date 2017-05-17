/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Disparity calculations implementation
//
//	Author(s): ben, Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/


#include <math.h>

#include <limits>
#include <fstream>

#include "Disparity.h"
#include "Common.h"
#include "RunningParameters.h"
#include "Utils.h"

cv::Mat DepthCalculator::GetDepthFromStereo(const cv::Mat& LeftImagePts, const cv::Mat& RightImagePts) const
{
	cv::Mat Pts4D, Pts3D;

	cv::triangulatePoints(leftCamera->K() * leftCamera->RT(), rightCamera->K() * rightCamera->RT(), LeftImagePts, RightImagePts, Pts4D);
	cv::convertPointsFromHomogeneous(Pts4D.t(), Pts3D);

	return Pts3D;
}


void AdjustDetectableRange(const RunningParameters& params, int& minDetectableDisparity, int& maxDetectableDisparity)
{
	int constDisparityDivider = 16;
	int constBitMask = constDisparityDivider - 1; // 0b1111

												  // opencv demands num of detectable disparities to be a multiple of 16
	int missingTo16 = (int)(unsigned)((minDetectableDisparity - maxDetectableDisparity) & constBitMask);
	int couldBeAdded = params.GetValue<int>(RobotParameters::LEFT_IMAGE_RESIZED_WIDTH) + minDetectableDisparity - maxDetectableDisparity;

	// adjust symmetrically in case we can't add enough pixels from both sides
	if (missingTo16 > couldBeAdded)
	{
		minDetectableDisparity += ((constDisparityDivider - missingTo16) / 2);
		maxDetectableDisparity -= ((constDisparityDivider - missingTo16) / 2);

		return;
	}

	// add missing pixels, since now there is enough room for the missing
	int maxToAddOnLeft = minDetectableDisparity;
	int maxToAddOnRight = params.GetValue<int>(RobotParameters::LEFT_IMAGE_RESIZED_WIDTH) - maxDetectableDisparity;

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


int RangeFinder::GetMinDisparity() const
{
	return 0;
}

int RangeFinder::GetMaxDisparity() const
{
	int minDisp, maxDisp = params.GetValue<int>(RobotParameters::RIGHT_IMAGE_RESIZED_WIDTH);
	AdjustDetectableRange(params, minDisp, maxDisp);

	return maxDisp;
}


void DisparityRangeFinder::DetectDisparities() const
{
	int RightImageResizedWidth = params.GetValue<int>(RobotParameters::RIGHT_IMAGE_RESIZED_WIDTH);
	int NumWidthPixelsToTest = params.GetValue<int>(RobotParameters::NumWidthPixelsToTest);
	int DisparitiesToTestRatio = params.GetValue<int>(RobotParameters::DisparitiesToTestRatio);

	int widthDistanceBetweenTestPoints = RightImageResizedWidth / NumWidthPixelsToTest;
	int heightDistanceBetweenTestPoints = params.GetHorizonHeight() / NumWidthPixelsToTest;
	int disparityDistance = widthDistanceBetweenTestPoints / DisparitiesToTestRatio;

	int actualWidthPoints = RightImageResizedWidth / widthDistanceBetweenTestPoints;
	int actualHeightPoints = params.GetHorizonHeight() / heightDistanceBetweenTestPoints;

	cv::Mat camLeftPts = cv::Mat::zeros(2, actualWidthPoints * actualHeightPoints * actualWidthPoints * DisparitiesToTestRatio, CV_64F),
		camRightPts = cv::Mat::zeros(2, actualWidthPoints * actualHeightPoints * actualWidthPoints * DisparitiesToTestRatio, CV_64F); //?

	int indx = 0;
	// prepare points to check
	for (int row = 0; row < params.GetHorizonHeight(); row += heightDistanceBetweenTestPoints)
	{
		for (int col = 0; col < RightImageResizedWidth; col += widthDistanceBetweenTestPoints)
		{
			for (int disparity = 0; disparity < col; disparity += disparityDistance)
			{
				camLeftPts.at<double>(0, indx) = col;
				camLeftPts.at<double>(1, indx) = row;
				camRightPts.at<double>(0, indx) = col - disparity;
				camRightPts.at<double>(1, indx) = row;

				++indx;
			}
		}
	}

	// get depth from points
	cv::Mat Pts3D = depthCalculator->GetDepthFromStereo(camLeftPts, camRightPts);

	int speed = params.GetValue<int>(RobotParameters::speed);
	int speedUnitInMm = params.GetValue<int>(RobotParameters::speedUnitInMm);

	double farthest_detectable_distance = speed * speedUnitInMm * params.GetValue<double>(RobotParameters::FARTHEST_DETECTABLE_RANGE);
	double closest_detectable_distance = speed * speedUnitInMm * params.GetValue<double>(RobotParameters::CLOSEST_DETECTABLE_RANGE);

	minDisparity = params.GetValue<int>(RobotParameters::RIGHT_CAMERA_FOV_WIDTH);
	maxDisparity = -1;

	for (int res = 0; res < Pts3D.rows; ++res)
	{
		double detectedDistance = Pts3D.row(res).at<double>(0, 2);
		if ((detectedDistance >= closest_detectable_distance) && (detectedDistance <= farthest_detectable_distance))
		{
			int disparity = camLeftPts.at<double>(0, res) - camRightPts.at<double>(0, res);

			if (disparity < minDisparity)
			{
				minDisparity = disparity;
			}

			if (disparity > maxDisparity)
			{
				maxDisparity = disparity;
			}
		}
	}

	AdjustDetectableRange(params, minDisparity, maxDisparity);

	lastCalculatedSpeed = speed;
}

int DisparityRangeFinder::GetMinDisparity() const
{
	if (params.GetValue<int>(RobotParameters::speed) != lastCalculatedSpeed)
	{
		DetectDisparities();
	}

	return minDisparity;
}

int DisparityRangeFinder::GetMaxDisparity() const
{
	if (params.GetValue<int>(RobotParameters::speed) != lastCalculatedSpeed)
	{
		DetectDisparities();
	}

	return maxDisparity;
}


cv::Mat DetectEdges(const RunningParameters& params, const cv::Mat& image)
{
	// allocate grayscale matrix of same size as given image
	cv::Mat edges = cv::Mat::zeros(image.size(), CV_8U);

	// smooth with Gaussian
	cv::GaussianBlur(image, edges, cv::Size(params.GAUSS_SIGMA, params.GAUSS_SIGMA), 0, 0, cv::BORDER_DEFAULT);
	// apply Laplacian kernel to detect edges
	cv::Laplacian(edges, edges, CV_8U, params.LAPLACE_KERN, 1, 0, cv::BORDER_DEFAULT);

	return edges;
}

cv::Mat ThresholdEdges(const RunningParameters& params, cv::Mat& edges)
{
	cv::Mat edgesThresh = cv::Mat::zeros(edges.size(), CV_8U);
	// i don't see why this is useful - may be making edges broader
	cv::medianBlur(edges, edges, 3);

	// make edges much more visible
	cv::threshold(edges, edgesThresh, params.THRESHOLD, params.GetValue<int>(RobotParameters::MAX_COLOR), cv::THRESH_BINARY);

	return edgesThresh;
}


void ExtractSignificantEdges(const RunningParameters& params, const cv::Mat& left, const cv::Mat& right, cv::Mat& edgesLeft, cv::Mat& edgesRight, cv::Mat& edgesLeftThresh, cv::Mat& edgesRightThresh)
{
	// do it on edges, not original images
	edgesLeft = DetectEdges(params, left);
	edgesRight = DetectEdges(params, right);

	edgesLeftThresh = ThresholdEdges(params, edgesLeft);
	edgesRightThresh = ThresholdEdges(params, edgesRight);

	cv::imshow(params.GetValue<std::string>(RobotParameters::LeftWindowName), edgesLeftThresh);
	cv::imshow(params.GetValue<std::string>(RobotParameters::RightWindowName), edgesRightThresh);
	cv::waitKey(1);
}

cv::Ptr<cv::StereoSGBM> CreateSGBM(const RunningParameters& params, int channels, int minDetectableDisparity, int numDisparities)
{
	int uniquenessRatio = 10;
	int speckleWindowSize = 100;
	int speckleRange = 2;
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

	// smoothness of disparity
	sgbm->setP1(8 * channels * sgbmWinSize * sgbmWinSize);
	sgbm->setP2(32 * channels * sgbmWinSize * sgbmWinSize);

	sgbm->setMinDisparity(minDetectableDisparity);
	sgbm->setNumDisparities(numDisparities);
	sgbm->setUniquenessRatio(uniquenessRatio);
	sgbm->setSpeckleWindowSize(speckleWindowSize);
	sgbm->setSpeckleRange(speckleRange);
	sgbm->setDisp12MaxDiff(disp12MaxDiff);

	// full mode matching
	sgbm->setMode(cv::StereoSGBM::MODE_HH);

#endif
	return sgbm;
}

cv::Mat SGBMDisparity::ComputeDisparity(const cv::Mat& left, const cv::Mat& right) const
{
	// set possible hit region
	int minDetectableDisparity = rangeFinder->GetMinDisparity(), 
		maxDetectableDisparity = rangeFinder->GetMaxDisparity();

	// make a divisable by 16 number of disparities
	int numDisparities = maxDetectableDisparity - minDetectableDisparity;

	// minDetectableDisparity = 0;
	cv::Ptr<cv::StereoSGBM> sgbm = CreateSGBM(params, left.channels(), minDetectableDisparity, numDisparities);

	cv::Mat edgesLeft;
	cv::Mat edgesRight;
	cv::Mat edgesLeftThresh;
	cv::Mat edgesRightThresh;

	ExtractSignificantEdges(params, left, right, edgesLeft, edgesRight, edgesLeftThresh, edgesRightThresh);

	cv::Mat disparity, disparity16S, maskedDisparity;
	// compute disparities
#if CV_VERSION_MAJOR < 3
	sgbm->operator()
#else
	sgbm->compute
#endif
	(left, right, disparity);

	// convert to human-acceptable visually
	disparity.convertTo(disparity16S, CV_16S, 1 / 16.);
	// now we are only interested in edges, so mask out non-edge stuff
	disparity16S.copyTo(maskedDisparity, (disparity16S > minDetectableDisparity) & (edgesLeftThresh >= params.THRESHOLD));

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


double CompareBlock(int i, int j, int k, int blockSize, const cv::Mat &left, const cv::Mat &right, const cv::Mat &edgeLeft, const cv::Mat &edgeRight)
{
	// ensures proper index access - no off image
	int halfBlockSize = (blockSize >> 1) << 1;

	int blockWidthLeft = std::min(halfBlockSize, std::min(j, k));
	int blockWidthRight = std::min(halfBlockSize, std::min(right.cols - j - 1, right.cols - k - 1));

	int blockHeightTop = std::min(halfBlockSize, i);
	int blockHeightBottom = std::min(halfBlockSize, right.rows - i - 1);

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

int ComputeBlockMatch(const RunningParameters& params, int i, int j, int blockSize, const cv::Mat& left, const cv::Mat& right, const cv::Mat& edgesLeft, const cv::Mat& edgesRight,
	const cv::Mat& leftBlockMap, const cv::Mat& rightBlockMap, const cv::Mat& disparity,
	std::vector<std::vector<int>>& matchMap, int minDetectableDisparity, int maxDetectableDisparity)
{
	// if block is not on the edge leave
	if (rightBlockMap.at<uchar>(i, j) == 0)
	{
		return 0;
	}

	//int col = (j == 0) ? j : j - 1;
	//int row = (i == 0) ? i : i - 1;

	std::vector<std::pair<double, int>> matches;
	double min = std::numeric_limits<double>::max();
	std::pair<double, int> firstErr, secondErr, thirdErr;
	int disp = 0;
	double leftErr(0), upErr(0), smoothnessErr(0), edgeProfileError(0);
	double err(0), blockErr(0);

	// in the closest range we could save few more cycles in not scanning till the end
	int stopK = j - minDetectableDisparity + params.OBST_THRESHOLD;
	int startK = std::max(0, j - maxDetectableDisparity - params.OBST_THRESHOLD);

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

			blockErr = CompareBlock(i, j, k, blockSize, right, left, edgesRight, edgesLeft);
			smoothnessErr = (leftErr + upErr) == 0 ? 1 : (leftErr + upErr);

			err = smoothnessErr * blockErr * edgeProfileError;

			if (abs(matchMap[i][k] - j) < 3 || matchMap[i][k] == 0)
			{

				matches.push_back(std::make_pair(err, j - k));
				if (err < min)
				{
					min = err;
					disp = j - k;
				}
			}
		}
	}

	sort(matches.begin(), matches.end());
	std::vector<std::pair<double, int>>::iterator it = matches.begin();


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


	if ((secondErr.first - firstErr.first) / secondErr.first < params.ERROR_THRESHOLD)
	{
		// some kind of pattern detection
		if (abs(secondErr.second - firstErr.second) <= params.DIST_THRESHOLD)
		{
			if (it == matches.end())
			{
				return disp;
			}

			thirdErr = *it;
			if ((thirdErr.first - firstErr.first) / thirdErr.first < params.ERROR_THRESHOLD)
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


cv::Mat HomeGrownDisparity::ComputeDisparity(const cv::Mat& left, const cv::Mat& right) const
{
	cv::Mat edgesLeft;
	cv::Mat edgesRight;
	cv::Mat edgesLeftThresh;
	cv::Mat edgesRightThresh;

	ExtractSignificantEdges(params, left, right, edgesLeft, edgesRight, edgesLeftThresh, edgesRightThresh);

	cv::Mat disparity = cv::Mat::zeros(edgesLeft.size(), CV_16S);

	std::vector<std::vector<int>> matchMap(disparity.rows, std::vector<int>(disparity.cols, 0));

	// set possible hit region
	int minDetectableDisparity = rangeFinder->GetMinDisparity(),
		maxDetectableDisparity = rangeFinder->GetMaxDisparity();


	// only down to horizon to suppress matching ground points
	for (int i = 0; i < params.GetHorizonHeight(); ++i)
	{
		// proceed with a horizontal scan
		for (int j = minDetectableDisparity; j < left.cols; ++j)
		{
            int disp = ComputeBlockMatch(params, i, j, params.nBlockSize, right, left, edgesRight, edgesLeft, edgesRightThresh, edgesLeftThresh, disparity, matchMap,
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

	cv::medianBlur(disparity, disparity, params.nBlockSize);

	return disparity;
}

