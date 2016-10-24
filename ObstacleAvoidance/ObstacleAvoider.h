#pragma once

#include <math.h>
#include "Common.h"

#ifndef M_PI
#define M_PI acos(-1.0)
#endif

class ObstacleAvoider
{
	RunningParameters& params;
	int savedTurningTime,
		savedSlopeMovementTime,
		savedForwardTime;
	int savedSpeed;

	static double ToSeconds(int timeInMs)
	{
		return timeInMs / 1000.0;
	}

	static int ToMilliseconds(double timeInSec)
	{
		return static_cast<int>(1000 * timeInSec);
	}

public:
	ObstacleAvoider(RunningParameters& params)
		: params(params)
	{
	}

	void DetectSafeTurningParameters(int& turningTime, int& slopeMovementTime, int& forwardTime)
	{
		const int NUM_BINS = 180 / 5 + 1;

		// return saved values in order not to recalculate every single time on constant speed
		if (params.speed != savedSpeed)
		{
			savedSpeed = params.speed;
			double actualSpeedInMm = savedSpeed * params.speedUnitInMm;

			// note there is always safe turning angle of 90 deg
			// we need to move at least half chassis + wheel + some safe distance to obstacle
			double minXDisplacement = Constants::OBSTACLE_DISTANCE + (Constants::CHASSIS_WIDTH + Constants::WHEEL_WIDTH) / 2;

			// those are interesting distances to the obstacle
			double maxYDisplacement = Constants::FARTHEST_DETECTABLE_RANGE * actualSpeedInMm;

			// work in 5 degree steps
			double xDisplacement[NUM_BINS], yDisplacement[NUM_BINS];
			double radianStep = M_PI * 5 / 180;
			double smallTurnRadius = Constants::CHASSIS_WIDTH / (params.spinMultiplierOnTurns - 1);

			// guess slope movement time in ms
			savedSlopeMovementTime = 300;

			bool goodEnough = false;
			bool adjustedTimeGuess = false;
			int goodAlternative;

			while (!goodEnough)
			{
				double slopeTimeInSeconds = ToSeconds(savedSlopeMovementTime);

				for (int i = 0; i < NUM_BINS; ++i)
				{
					xDisplacement[i] = (params.spinMultiplierOnTurns + 1) * smallTurnRadius * (1 - cos(i * radianStep)) + slopeTimeInSeconds * actualSpeedInMm * sin(i * radianStep);
					yDisplacement[i] = (params.spinMultiplierOnTurns + 1) * smallTurnRadius * sin(i * radianStep) + slopeTimeInSeconds * actualSpeedInMm * cos(i * radianStep);
				}

				// try to peak a non-severe angle
				for (int i = 0; i < NUM_BINS / 2; ++i)
				{
					if (xDisplacement[i] > minXDisplacement && yDisplacement[i] < maxYDisplacement)
					{
						// alternative found
						goodAlternative = i;
						goodEnough = true;
						break;
					}
				}

				if (!goodEnough)
				{
					if (adjustedTimeGuess)
					{
						goodAlternative = NUM_BINS - 1;
						goodEnough = true;
						for (int i = 0; i < NUM_BINS - 1; ++i)
						{
							if (xDisplacement[i] > minXDisplacement && yDisplacement[i] > maxYDisplacement)
							{
								// alternative found
								goodAlternative = i;
								break;
							}
						}
					}
					else
					{
						savedSlopeMovementTime += 200;
						adjustedTimeGuess = true;
					}
				}
			}

			// once again differential turning speeds
			// turning radius multiplied by angle divided by speed
			savedTurningTime = ToMilliseconds(4 * smallTurnRadius * (goodAlternative * radianStep) / actualSpeedInMm);
			// need yet to cover some distance in y direction [to the north]
			savedForwardTime = ToMilliseconds((Constants::OBSTACLE_LENGTH + maxYDisplacement - yDisplacement[goodAlternative]) / actualSpeedInMm);
		}

		turningTime = savedTurningTime;
		slopeMovementTime = savedSlopeMovementTime;
		forwardTime = savedForwardTime;
	}
};