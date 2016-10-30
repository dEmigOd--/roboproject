#include <iostream>
#include "RobotManeuver.h"
#include "RobotVision.h"
#ifdef __linux__ 
#include <unistd.h>
#elif _WIN32
#include <io.h>
#endif
#include "Thresholds.h"
#include "Logger.h"
#include "Common.h"

#define KEY_UP 65362
#define KEY_DOWN 65364
#define KEY_LEFT 65361
#define KEY_RIGHT 65363
#define ESC 27
#define KEY_LEFT_SHARP 65460 // KP_4
#define KEY_STOP 65461 // KP_5
#define KEY_RIGHT_SHARP 65462 // KP_6

const std::string RunningParameters::BaseName = "RobotFov";
const std::string RunningParameters::ImageExtensionName = ".png";
const std::string RunningParameters::MatExtensionName = ".mat";
const std::string RunningParameters::Separator = "/";
const std::string RunningParameters::LeftSuffix = "_" + Constants::LeftWindowName + "_";
const std::string RunningParameters::RightSuffix = "_" + Constants::RightWindowName + "_";
const std::string RunningParameters::ImagesDirName = "images";
const std::string RunningParameters::OutputDirName = "output";

int THRESHOLD = 30;
int DENSITY = 5;
int OBST_THRESHOLD = 0;
int NPTS = 12;
int EVENT_THRESHOLD = 1;
double ERROR_THRESHOLD = 0.8;
int GAUSS_SIGMA = 5;
int LAPLACE_KERN = 3;
int DIST_THRESHOLD = 8;

int tresh = 80;
int odd = 3;

bool smoothTurn = true;

std::mutex syncronizer;
// create logger for main part
el::Logger* logger = Logger::GetLogger("main");


struct CallbackData
{
	cv::Mat disparity;
	RunningParameters params;
};

bool CaseInsensitiveStringCompare(const std::string& str1, const std::string& str2)
{
	if (str1.size() != str2.size())
	{
		return false;
	}
	for (std::string::const_iterator c1 = str1.begin(), c2 = str2.begin(); c1 != str1.end(); ++c1, ++c2)
	{
		if (tolower(*c1) != tolower(*c2))
		{
			return false;
		}
	}
	return true;
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

void CallBackFunc(int event, int j, int i, int flags, void* userdata)
{
	CallbackData* data = (CallbackData*)userdata;
	double dist = 0;
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
		dist = Constants::OPTICAL_AXIS_DISTANCE * data->params.LEFT_CAMERA_FOV_WIDTH / (x * 2 * Constants::CAMERA_ANGLE_WIDTH_MULTIPLIER);

		std::cout << "position (" << row << ", " << col << "). Distance is "
			<< dist << " mm , disparity is " << x << " pixels" << std::endl;
	}
}

RunningParameters ParseInputArguments(int argc, char* argv[])
{
	RunningParameters params;

	for (int currentArgN = 1; currentArgN < argc; ++currentArgN)
	{
		if (CaseInsensitiveStringCompare(argv[currentArgN], "--record"))
		{
			params.SetRecordMode();
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--replay"))
		{
			params.SetReplayMode();
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--workDir"))
		{
			params.wrkDir = argv[++currentArgN];
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--maxRecord"))
		{
			params.numToRecord = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--frequency"))
		{
			params.frequency = std::chrono::milliseconds(atoi(argv[++currentArgN]));
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--speed"))
		{
			params.speed = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--turnMultiplier"))
		{
			params.spinMultiplierOnTurns = atof(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--dontMove"))
		{
			params.shouldMove = false;
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--debugCameras"))
		{
			params.debugCameras = true;
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--debugMath"))
		{
			params.debugMath = true;
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--width"))
		{
			int width = atoi(argv[++currentArgN]);
			params.LEFT_CAMERA_FOV_WIDTH = width;
			params.RIGHT_CAMERA_FOV_WIDTH = width;
			params.LEFT_IMAGE_RESIZED_WIDTH = params.LEFT_CAMERA_FOV_WIDTH / Constants::RESIZE_FACTOR;
			params.RIGHT_IMAGE_RESIZED_WIDTH = params.RIGHT_CAMERA_FOV_WIDTH / Constants::RESIZE_FACTOR;
			logger->trace("Width read: %v", width);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--height"))
		{
			int width = atoi(argv[++currentArgN]);
			params.LEFT_CAMERA_FOV_HEIGHT = width;
			params.RIGHT_CAMERA_FOV_HEIGHT = width;
			params.LEFT_IMAGE_RESIZED_HEIGHT = params.LEFT_CAMERA_FOV_HEIGHT / Constants::RESIZE_FACTOR;
			params.RIGHT_IMAGE_RESIZED_HEIGHT = params.RIGHT_CAMERA_FOV_HEIGHT / Constants::RESIZE_FACTOR;
			logger->trace("Height read: %v", width);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--leftCameraIndex"))
		{
			params.leftCameraIdx = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--rightCameraIndex"))
		{
			params.leftCameraIdx = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--useAmatuerForMatching"))
		{
			params.useOpenCVAlgorihmForMatching = MatchingAlgorithm::OWN_BLOCKMATCHING;
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--blockSize"))
		{
			params.nBlockSize = atoi(argv[++currentArgN]);
			continue;
		}

		if (CaseInsensitiveStringCompare(argv[currentArgN], "--ignoreFarPoints"))
		{
			params.ignoreFarPointsInSecAway = atof(argv[++currentArgN]);
			continue;
		}

	}

	return params;
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

				if (eventsCounter >= EVENT_THRESHOLD)
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
					smoothTurn ? robotManeuver.SmoothLeft() : robotManeuver.Left();
					break;
				case KEY_LEFT_SHARP:
					robotManeuver.Turn90Deg(RobotManeuver::LEFT);
					break;
				case KEY_RIGHT:
					smoothTurn ? robotManeuver.SmoothRight() : robotManeuver.Right();
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
				smoothTurn = true;
				break;
			case 'y':
				smoothTurn = false;
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

	//// read config from file
	//RobotConfig appConfig;
	//appConfig.ReloadConfig();

	logger->info("------------- New Session -------------");

	CallbackData data;
	data.params = ParseInputArguments(argc, argv);

	cv::namedWindow(Constants::MatchesWindowName, CV_WINDOW_NORMAL);
	cv::namedWindow(Constants::RightWindowName, CV_WINDOW_NORMAL);
	cv::namedWindow(Constants::LeftWindowName, CV_WINDOW_NORMAL);

	cv::resizeWindow(Constants::LeftWindowName, 
		Constants::IMAGE_STRETCH_FACTOR * data.params.LEFT_IMAGE_RESIZED_WIDTH / Constants::IMAGE_COMPRESS_FACTOR,
		Constants::IMAGE_STRETCH_FACTOR * data.params.LEFT_IMAGE_RESIZED_HEIGHT / Constants::IMAGE_COMPRESS_FACTOR);
	cv::resizeWindow(Constants::RightWindowName, 
		Constants::IMAGE_STRETCH_FACTOR * data.params.RIGHT_IMAGE_RESIZED_WIDTH / Constants::IMAGE_COMPRESS_FACTOR,
		Constants::IMAGE_STRETCH_FACTOR * data.params.RIGHT_IMAGE_RESIZED_HEIGHT / Constants::IMAGE_COMPRESS_FACTOR);

	cv::createTrackbar("THRESHOLD", Constants::MatchesWindowName, &THRESHOLD, 120, TrackBarFunc);
	cv::createTrackbar("DENSITY", Constants::MatchesWindowName, &DENSITY, 20, TrackBarFunc);
	cv::createTrackbar("NPTS", Constants::MatchesWindowName, &NPTS, 100, TrackBarFunc);
	cv::createTrackbar("EVENT_THRESHOLD", Constants::MatchesWindowName, &EVENT_THRESHOLD, 10, TrackBarFunc);
	cv::createTrackbar("BLOCK_SIZE", Constants::MatchesWindowName, &data.params.nBlockSize, 20, TrackBarFuncOdd, &data.params.nBlockSize);
	cv::createTrackbar("ERROR_THRESHOLD", Constants::MatchesWindowName, &tresh, Constants::CONVERT_TO_PERCENT, TrackBarFuncTresh, &ERROR_THRESHOLD);
	cv::createTrackbar("GAUSS_SIGMA", Constants::MatchesWindowName, &GAUSS_SIGMA, 10, TrackBarFuncOdd, &GAUSS_SIGMA);
	cv::createTrackbar("LAPLACE_KERN", Constants::MatchesWindowName, &LAPLACE_KERN, 10, TrackBarFuncOdd, &LAPLACE_KERN);
	cv::createTrackbar("OBST_THRESHOLD", Constants::MatchesWindowName, &OBST_THRESHOLD, 25, TrackBarFunc);

	RobotManeuver robotManeuver(data.params);
	RobotVision robotVision(syncronizer, data.params);

	cv::setMouseCallback(Constants::MatchesWindowName, CallBackFunc, (void*) &data);

	std::unique_lock<std::mutex> locker(syncronizer);
	robotVision.condition.wait(locker, [&robotVision]
	{
		return robotVision.initialized;
	});

	return RunRobot(robotManeuver, robotVision, data);
}
