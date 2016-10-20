#pragma once
#ifndef ROBOT_CONFIGURATION
#define ROBOT_CONFIGURATION

#include "libconfig++.h"
#include <map>

class RobotConfig
{
public:
	enum RobotConfigName
	{
		WINDOWS_NAME_MATCHES,
		WINDOWS_NAME_LEFT,
		WINDOWS_NAME_RIGHT,

		TRACKBAR_NAME_THRESHOLD,
		TRACKBAR_NAME_DENSITY,
		TRACKBAR_NAME_NPTS,
		TRACKBAR_NAME_EVENT_THRESHOLD,
		TRACKBAR_NAME_MEDIAN_BLUR_WINDOW,
		TRACKBAR_NAME_ERROR_THRESHOLD,
		TRACKBAR_NAME_GAUSS_SIGMA,
		TRACKBAR_NAME_LAPLACE_KERN,
		TRACKBAR_NAME_OBST_THRESHOLD,

		TRACKBAR_MAXVALUE_THRESHOLD,
		TRACKBAR_MAXVALUE_DENSITY,
		TRACKBAR_MAXVALUE_NPTS,
		TRACKBAR_MAXVALUE_EVENT_THRESHOLD,
		TRACKBAR_MAXVALUE_MEDIAN_BLUR_WINDOW,
		TRACKBAR_MAXVALUE_ERROR_THRESHOLD,
		TRACKBAR_MAXVALUE_GAUSS_SIGMA,
		TRACKBAR_MAXVALUE_LAPLACE_KERN,
		TRACKBAR_MAXVALUE_OBST_THRESHOLD,
	};

private:

	libconfig::Config appConfig;
	static std::map<RobotConfigName, std::string> knownConfigs;

	static void AddConfig(RobotConfigName key, const std::string& value)
	{
		knownConfigs[key] = value;
	}

public:

	static void InitConfigs()
	{
		AddConfig(WINDOWS_NAME_MATCHES, "application.windows.matches.name");
		AddConfig(WINDOWS_NAME_LEFT, "application.windows.left.name");
		AddConfig(WINDOWS_NAME_RIGHT, "application.windows.right.name");

		AddConfig(TRACKBAR_NAME_THRESHOLD, "application.windows.matches.trackbars.threshold.name");
		AddConfig(TRACKBAR_NAME_DENSITY, "application.windows.matches.trackbars.density.name");
		AddConfig(TRACKBAR_NAME_NPTS, "application.windows.matches.trackbars.npts.name");
		AddConfig(TRACKBAR_NAME_EVENT_THRESHOLD, "application.windows.matches.trackbars.eventthreshold.name");
		AddConfig(TRACKBAR_NAME_MEDIAN_BLUR_WINDOW, "application.windows.matches.trackbars.medianblurwindow.name");
		AddConfig(TRACKBAR_NAME_ERROR_THRESHOLD, "application.windows.matches.trackbars.errorthreshold.name");
		AddConfig(TRACKBAR_NAME_GAUSS_SIGMA, "application.windows.matches.trackbars.gausssigma.name");
		AddConfig(TRACKBAR_NAME_LAPLACE_KERN, "application.windows.matches.trackbars.laplacekern.name");
		AddConfig(TRACKBAR_NAME_OBST_THRESHOLD, "application.windows.matches.trackbars.obstthreshold.name");

		AddConfig(TRACKBAR_MAXVALUE_THRESHOLD, "application.windows.matches.trackbars.threshold.maxvalue");
		AddConfig(TRACKBAR_MAXVALUE_DENSITY, "application.windows.matches.trackbars.density.maxvalue");
		AddConfig(TRACKBAR_MAXVALUE_NPTS, "application.windows.matches.trackbars.npts.maxvalue");
		AddConfig(TRACKBAR_MAXVALUE_EVENT_THRESHOLD, "application.windows.matches.trackbars.eventthreshold.maxvalue");
		AddConfig(TRACKBAR_MAXVALUE_MEDIAN_BLUR_WINDOW, "application.windows.matches.trackbars.medianblurwindow.maxvalue");
		AddConfig(TRACKBAR_MAXVALUE_ERROR_THRESHOLD, "application.windows.matches.trackbars.errorthreshold.maxvalue");
		AddConfig(TRACKBAR_MAXVALUE_GAUSS_SIGMA, "application.windows.matches.trackbars.gausssigma.maxvalue");
		AddConfig(TRACKBAR_MAXVALUE_LAPLACE_KERN, "application.windows.matches.trackbars.laplacekern.maxvalue");
		AddConfig(TRACKBAR_MAXVALUE_OBST_THRESHOLD, "application.windows.matches.trackbars.obstthreshold.maxvalue");
	}

	void ReloadConfig()
	{
		// read config from file
		appConfig.readFile("robot.config");
	}

	template<typename TType>
	TType lookup(RobotConfigName key) const
	{
		return (TType) appConfig.lookup(knownConfigs[key]);
	}

	libconfig::Setting& lookup(RobotConfigName key) const
	{
		return appConfig.lookup(knownConfigs[key]);
	}
};

#endif
