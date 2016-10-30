#pragma once

#include <iostream>
#include <string>
#include "OpenCv.h"

class CalibratedConfig
{
	const std::string calibrationDataFileName = "out_camera_data.xml";

	int imgWidth, imgHeight;
	double actualWidthMultiplier = 1.0, 
		actualHeightMultiplier = 1.0;
	mutable bool resized = true;
	cv::Mat KMatrix, RTMatrix, distortion;

	void ReadCameraData()
	{
		cv::FileStorage cameraSettings(calibrationDataFileName, cv::FileStorage::READ);

		if (!cameraSettings.isOpened()) throw std::runtime_error("Camera calibration datafile not found");

		cameraSettings["image_width"] >> imgWidth;
		cameraSettings["image_height"] >> imgHeight;

		cameraSettings["camera_matrix"] >> KMatrix;
		cameraSettings["distortion_coefficients"] >> distortion;
	}

public:
	// cameras are parallel and reference frame is in the center of left camera !!
	CalibratedConfig(const cv::Vec3d& cameraPosition)
	{
		ReadCameraData();
		cv::hconcat(cv::Mat::eye(3, 3, CV_64F), cameraPosition, RTMatrix);
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
			cv::Mat resolutionChangeMatrix = cv::Mat::zeros(3, 3, CV_64F);
			resolutionChangeMatrix.diag().setTo(cv::Vec3d({ actualWidthMultiplier, actualHeightMultiplier, 1.0 }));
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

