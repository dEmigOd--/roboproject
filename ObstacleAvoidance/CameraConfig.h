#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//  IMPORTANT: READ BEFORE USING.
//
//	CalibratedConfig class 
//	implements abstraction layer to extract the physical information about the camera
//	
//	*	you should calibrate your cameras (separately) first (using OpenCV for example) to extract
//		intrinsic and extrinsic camera parameters
//	*	use higher resolutions during calibration to get lower errors
//	*	use a lot of images (move camera, if real-time)
//	
//	see RunningParameters.CameraCalibrationDataFileName for file name/file extension
//	for example: out_camera_data_0.xml for camera id-ed 0 in calibration dir
//	
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include <iostream>
#include <string>

#include "OpenCv.h"
#include "Common.h"

class CalibratedConfig
{
	int imgWidth, imgHeight;
	double actualWidthMultiplier = 1.0, 
		actualHeightMultiplier = 1.0;
	mutable bool resized = true;
	cv::Mat KMatrix, RTMatrix, distortion;

	void ReadCameraData(const RunningParameters& params, int cameraId)
	{
		cv::FileStorage cameraSettings(params.BuildCameraConfigFilename(cameraId), cv::FileStorage::READ);

		if (!cameraSettings.isOpened()) throw std::runtime_error("Camera calibration datafile not found");

		cameraSettings["image_width"] >> imgWidth;
		cameraSettings["image_height"] >> imgHeight;

		cameraSettings["camera_matrix"] >> KMatrix;
		cameraSettings["distortion_coefficients"] >> distortion;
	}

public:
	// Rreference frame is in the middle between two cameras !!
	CalibratedConfig(const RunningParameters& params, int cameraId, const cv::Vec3d& cameraPosition)
	{
		ReadCameraData(params, cameraId);
		cv::hconcat(cv::Mat::eye(3, 3, CV_64F), -cameraPosition, RTMatrix);
	}

	int ResolutionWidth() const
	{
		return imgWidth;
	}

	int ResolutionHeight() const
	{
		return imgHeight;
	}

	const cv::Mat& K() const
	{
		static cv::Mat adjustedKMatrix;

		if (resized)
		{
			// adjust K for resolution, as calibration was done at very specific one
			double st[3][3] = { { actualWidthMultiplier, 0.0, 0.0 },{ 0.0, actualHeightMultiplier, 0.0 },{ 0.0, 0.0, 1.0 } };

			cv::Mat resolutionChangeMatrix = cv::Mat(3, 3, CV_64F, st);
			resized = false;

			adjustedKMatrix = resolutionChangeMatrix * KMatrix;
		}

		return adjustedKMatrix;
	}

	const cv::Mat& RT() const
	{
		return RTMatrix;
	}

	const cv::Mat& Distortion() const
	{
		return distortion;
	}
	
	void SetResolutionWidthTo(double actualWidth)
	{
		actualWidthMultiplier = actualWidth / imgWidth;
		resized = true;
	}

	void SetResolutionHeightTo(double actualHeight)
	{
		actualHeightMultiplier = actualHeight / imgHeight;
		resized = true;
	}
};
