/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	CameraModel class
//	implements resolution property hooking for VideoCapture
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include "CameraModel.h"

bool CameraModel::set(int propId, double value)
{
	switch (propId)
	{
		case CV_CAP_PROP_FRAME_WIDTH:
			SetResolutionWidthTo(value);
			break;
		case CV_CAP_PROP_FRAME_HEIGHT:
			SetResolutionHeightTo(value);
			break;
		default:
			break;
	}

	if(fallThrough)
		return VideoCapture::set(propId, value);

	return true;
}
