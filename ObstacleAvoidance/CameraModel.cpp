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
