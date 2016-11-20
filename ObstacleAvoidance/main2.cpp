/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//  IMPORTANT: READ BEFORE USING.
//
//	main
//	implements control loop for robot
//
//	*	creates UI windows
//	*	organizes user interaction
//	*	controls robot movement, in terms of giving response to detected obstacle
//
//	Author(s): ben, Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include <iostream>
#ifdef __linux__ 
#include <unistd.h>
#elif _WIN32
#include <io.h>
#endif

#include "Logger.h"
#include "Common.h"
#include "RobotInputParser.h"
#include "RobotManeuver.h"
#include "RobotVision.h"

// keyboard keys, which controls the robot
#define KEY_UP 65362
#define KEY_DOWN 65364
#define KEY_LEFT 65361
#define KEY_RIGHT 65363
#define ESC 27
#define KEY_LEFT_SHARP 65460 // KP_4
#define KEY_STOP 65461 // KP_5
#define KEY_RIGHT_SHARP 65462 // KP_6

std::mutex syncronizer;
// create logger for main part
el::Logger* logger = Logger::GetLogger("main");

struct CallbackData
{
	cv::Mat disparity;
	RunningParameters params;
	std::shared_ptr<DepthCalculator> depthCalculator;
};

void InitializeUserControllerValues(RunningParameters& params)
{
	params.THRESHOLD = 30;
	params.DENSITY = 5;
	params.OBST_THRESHOLD = 0;
	params.NPTS = 12;
	params.EVENT_THRESHOLD = 1;
	params.ERROR_THRESHOLD = 0.8;
	params.GAUSS_SIGMA = 5;
	params.LAPLACE_KERN = 3;
	params.DIST_THRESHOLD = 8;
	params.tresh = 80;
	params.smoothTurn = true;
}

void TrackBarFunc(int num, void* x)
{
}

void TrackBarFuncTresh(int num, void* x)
{
	*(double*) x = (double)num / Constants::CONVERT_TO_PERCENT;
}

void TrackBarFuncDouble(int num, void* x)
{
	*(double*)x = (double)num;
}

void TrackBarFuncOdd(int num, void* x)
{
	int* param = (int*)x;
	*param = num | 0x1;
}

double GetDistanceToPoint(const DepthCalculator& depthCalculator, int row, int col, int disparity)
{
	cv::Mat camLeftPts = cv::Mat::zeros(2, 1, CV_64F),
		camRightPts = cv::Mat::zeros(2, 1, CV_64F); //?

	camLeftPts.at<double>(0, 0) = col;
	camLeftPts.at<double>(1, 0) = row;
	camRightPts.at<double>(0, 0) = col - disparity;
	camRightPts.at<double>(1, 0) = row;

	cv::Mat Pts3D = depthCalculator.GetDepthFromStereo(camLeftPts, camRightPts);

	return Pts3D.row(0).at<double>(0, 2);
}

void CallBackFunc(int event, int j, int i, int flags, void* userdata)
{
	CallbackData* data = (CallbackData*)userdata;
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		int row = i;
		int col = j;
		if (row > data->disparity.rows || col > data->disparity.cols || col < 0)
		{
			std::cout << "Click on left image" << std::endl;
			return;
		}

		int x = (int)(data->disparity.at<short>(row, col));

		std::cout << "position (" << row << ", " << col << "). Distance is "
			<< GetDistanceToPoint(*data->depthCalculator, row, col, x) << " mm , disparity is " << x << " pixels" << std::endl;
	}
}

int RunRobot(RobotManeuver& robotManeuver, RobotVision& robotVision, CallbackData& data)
{
	int eventsCounter = 0;
	bool navMode(true);
	int key = cv::waitKey(0);

	if (data.params.debugCameras)
	{
		data.params.startedProcessing = true;
	}

	while (true)
	{

		RobotVision::SideEnum eSide = RobotVision::NONE;

		eSide = robotVision.ObstaclePresent();

		data.disparity = robotVision.GetCurrentDisparity();

		if (!navMode)
		{
			if (eSide != RobotVision::NONE)
			{
				eventsCounter++;
				logger->debug("Current event counter is: %v", eventsCounter);

				if (eventsCounter >= data.params.EVENT_THRESHOLD)
				{

					switch (eSide)
					{
					case RobotVision::LEFT:
						robotManeuver.ManeuverRight();
						break;
					case RobotVision::RIGHT:
						robotManeuver.ManeuverLeft();
						break;
					default:
						break;
					}
					eventsCounter = 0;
				}
			}
			else
			{
				eventsCounter = 0;
				robotManeuver.Forward();
			}
		}
		else
		{
			switch (key)
			{
				case 'w':
				case KEY_UP:
					robotManeuver.Forward();
					break;
				case KEY_DOWN:
				case 's':
					robotManeuver.Backward();
					break;
				case KEY_STOP:
				case 32: // space
					robotManeuver.ImmediateStop();
					break;
				case KEY_LEFT:
					data.params.smoothTurn ? robotManeuver.SmoothLeft() : robotManeuver.Left();
					break;
				case KEY_LEFT_SHARP:
					robotManeuver.Turn90Deg(RobotManeuver::LEFT);
					break;
				case KEY_RIGHT:
					data.params.smoothTurn ? robotManeuver.SmoothRight() : robotManeuver.Right();
					break;
				case KEY_RIGHT_SHARP:
					robotManeuver.Turn90Deg(RobotManeuver::RIGHT);
					break;
				case 'a':
					robotManeuver.ManeuverLeft();
					break;
				case 'd':
					robotManeuver.ManeuverRight();
					break;
				case ESC:
					return 0;
				default:
					break;
			}
		}

		key = cv::waitKey(10);
		if (key != -1)
		{
			logger->debug("Key read: %v", key);
		}

		switch (key)
		{
			// switch to manual
			case 'k': // 107
				robotManeuver.Stop();
				navMode = true;
				if (data.params.debugCameras)
				{
					data.params.startedProcessing = false;
				}
			break;
			case 'r':
				data.params.PrepareForNextImage();
				WriteCSV(data.params.BuildMatrixName("Disparity"), data.disparity);
				break;
			case ESC:
				robotManeuver.Stop();
				return 0;
			//switch to automatic
			case 'l': //108
				if (navMode)
				{
					eventsCounter = 0;
					navMode = false;
					robotManeuver.Forward();
					if (data.params.debugCameras)
					{
						data.params.startedProcessing = true;
					}
				}
				break;
			case 't': // smooth turns
				data.params.smoothTurn = true;
				break;
			case 'y': // hard turns
				data.params.smoothTurn = false;
			default:
				break;
		}
	}

	return 0;
}

int main(int argc, char* argv[])
{
	// initialize logging in the program
	Logger::Initialize();

	logger->info("------------- New Session -------------");

	CallbackData data;
	data.params = RobotInputParser::ParseInputArguments(argc, argv);

	InitializeUserControllerValues(data.params);

	cv::namedWindow(Constants::MatchesWindowName, CV_WINDOW_NORMAL);
	cv::namedWindow(Constants::RightWindowName, CV_WINDOW_NORMAL);
	cv::namedWindow(Constants::LeftWindowName, CV_WINDOW_NORMAL);

	cv::resizeWindow(Constants::LeftWindowName, 
		Constants::IMAGE_STRETCH_FACTOR * data.params.LEFT_IMAGE_RESIZED_WIDTH / Constants::IMAGE_COMPRESS_FACTOR,
		Constants::IMAGE_STRETCH_FACTOR * data.params.LEFT_IMAGE_RESIZED_HEIGHT / Constants::IMAGE_COMPRESS_FACTOR);
	cv::resizeWindow(Constants::RightWindowName, 
		Constants::IMAGE_STRETCH_FACTOR * data.params.RIGHT_IMAGE_RESIZED_WIDTH / Constants::IMAGE_COMPRESS_FACTOR,
		Constants::IMAGE_STRETCH_FACTOR * data.params.RIGHT_IMAGE_RESIZED_HEIGHT / Constants::IMAGE_COMPRESS_FACTOR);

	cv::createTrackbar("THRESHOLD", Constants::MatchesWindowName, &data.params.THRESHOLD, 120, TrackBarFunc);
	cv::createTrackbar("DENSITY", Constants::MatchesWindowName, &data.params.DENSITY, 20, TrackBarFunc);
	cv::createTrackbar("NPTS", Constants::MatchesWindowName, &data.params.NPTS, 100, TrackBarFunc);
	cv::createTrackbar("EVENT_THRESHOLD", Constants::MatchesWindowName, &data.params.EVENT_THRESHOLD, 10, TrackBarFunc);
	cv::createTrackbar("BLOCK_SIZE", Constants::MatchesWindowName, &data.params.nBlockSize, 20, TrackBarFuncOdd, &data.params.nBlockSize);
	cv::createTrackbar("ERROR_THRESHOLD", Constants::MatchesWindowName, &data.params.tresh, Constants::CONVERT_TO_PERCENT, TrackBarFuncTresh, &data.params.ERROR_THRESHOLD);
	cv::createTrackbar("GAUSS_SIGMA", Constants::MatchesWindowName, &data.params.GAUSS_SIGMA, 10, TrackBarFuncOdd, &data.params.GAUSS_SIGMA);
	cv::createTrackbar("LAPLACE_KERN", Constants::MatchesWindowName, &data.params.LAPLACE_KERN, 10, TrackBarFuncOdd, &data.params.LAPLACE_KERN);
	cv::createTrackbar("OBST_THRESHOLD", Constants::MatchesWindowName, &data.params.OBST_THRESHOLD, 25, TrackBarFunc);

	RobotManeuver robotManeuver(data.params);
	RobotVision robotVision(syncronizer, data.params);

	cv::setMouseCallback(Constants::MatchesWindowName, CallBackFunc, (void*) &data);

	std::unique_lock<std::mutex> locker(syncronizer);
	robotVision.condition.wait(locker, [&robotVision]
	{
		return robotVision.initialized;
	});

	// here robotVision initialized already
	data.depthCalculator = robotVision.depthCalculator;
	return RunRobot(robotManeuver, robotVision, data);
}
