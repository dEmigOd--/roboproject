#include "UI.h"
#include <iostream>
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

int THRESHOLD = 30;
int DENSITY = 1;
int OBST_THRESHOLD = 11;
int NPTS = 12;
int EVENT_THRESHOLD = 1;
int MEDIAN_BLUR_WINDOW = 3;
double ERROR_THRESHOLD = 0.8;
int GAUSS_SIGMA = 5;
int LAPLACE_KERN = 3;
int DIST_THRESHOLD = 8;

int tresh = 80;
int odd = 3;

int  ERROR_THRESHOLD_MAX = 100;


void TrackBarFunc(int num, void* x)
{
	DelayedConfigEntry* underlying = (DelayedConfigEntry*) x;

	underlying->OnChange(num);
	underlying->InvokeCallback(num);
}

void TrackBarFuncTresh(int num, void* x)
{
	ERROR_THRESHOLD = (double)tresh / ERROR_THRESHOLD_MAX;
}

void TrackBarFuncOdd(int num, void* x)
{
	int* param = (int*)x;
	*param = odd % 2 ? odd : odd + 1;
}

void CallBackFunc(int event, int j, int i, int flags, void* userdata)
{
	cv::Mat* disparity = ((cv::Mat*)userdata);
	double dist = 0;
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		int row = i;
		int col = j - disparity->cols;
		if (row > disparity->rows || col > disparity->cols || col < 0)
		{
			std::cout << "Click on right image" << std::endl;
			return;
		}
		int x = (int)(disparity->at<uchar>(row, col)) * 4;
		dist = 0.0543 * pow(x, 2) - 7.3872 * x + 277.78;

		std::cout << "position (" << row << ", " << col << ")" << "Distance is "
			<< dist << " , disparity is " << x << std::endl;
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}


void UI::CreateTrackBar(const std::string& wndName, DelayedConfigEntry* dConfigEntry)
{
	allocatedElements.push_back(std::unique_ptr<void>(dConfigEntry));

	cv::createTrackbar
		(
			_config->lookup<std::string>(dConfigEntry->_name),
			wndName, &dConfigEntry->retValue,
			_config->lookup<int>(dConfigEntry->_maxValueName),
			TrackBarFunc,
			(--allocatedElements.end())->get()
			);
}

void UI::CreateUI()
{
	cv::namedWindow(_config->lookup<std::string>(RobotConfig::WINDOWS_NAME_MATCHES), CV_WINDOW_NORMAL);
	cv::namedWindow(_config->lookup<std::string>(RobotConfig::WINDOWS_NAME_LEFT), CV_WINDOW_NORMAL);
	cv::namedWindow(_config->lookup<std::string>(RobotConfig::WINDOWS_NAME_RIGHT), CV_WINDOW_NORMAL);

	std::string windowMatchesName = _config->lookup(RobotConfig::WINDOWS_NAME_MATCHES);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_THRESHOLD),
			windowMatchesName, 
			&THRESHOLD, 
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_THRESHOLD),
			TrackBarFunc
		);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_DENSITY),
			windowMatchesName,
			&DENSITY,
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_DENSITY),
			TrackBarFunc
			);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_NPTS),
			windowMatchesName,
			&NPTS,
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_NPTS),
			TrackBarFunc
			);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_EVENT_THRESHOLD),
			windowMatchesName,
			&EVENT_THRESHOLD,
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_EVENT_THRESHOLD),
			TrackBarFunc
			);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_MEDIAN_BLUR_WINDOW),
			windowMatchesName,
			&odd,
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_MEDIAN_BLUR_WINDOW),
			TrackBarFuncOdd, 
			&MEDIAN_BLUR_WINDOW
			);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_ERROR_THRESHOLD),
			windowMatchesName,
			&tresh,
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_ERROR_THRESHOLD),
			TrackBarFuncTresh
			);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_GAUSS_SIGMA),
			windowMatchesName,
			&odd,
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_GAUSS_SIGMA),
			TrackBarFuncOdd, 
			&GAUSS_SIGMA
			);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_LAPLACE_KERN),
			windowMatchesName,
			&odd,
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_LAPLACE_KERN),
			TrackBarFuncOdd, 
			&LAPLACE_KERN
			);

	cv::createTrackbar
		(
			_config->lookup<std::string>(RobotConfig::TRACKBAR_NAME_OBST_THRESHOLD),
			windowMatchesName,
			&OBST_THRESHOLD,
			_config->lookup<int>(RobotConfig::TRACKBAR_MAXVALUE_OBST_THRESHOLD),
			TrackBarFunc
			);

}

