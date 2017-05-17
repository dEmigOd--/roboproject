#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//  IMPORTANT: READ BEFORE USING.
//
//	ObstacleAvoider class
//	implements decision parameter solver for avoiding a close obstacle
//
//	*	there are certain guesses about parameters, so check them out
//	*	speed is taken into account
//	*	uses a pocket-on-the-road avoidance technique
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include <math.h>

#include "Common.h"

#ifndef M_PI
#define M_PI acos(-1.0)
#endif

#define MS_IN_SECOND 1000
#define MS_IN_SECOND_D (double)(MS_IN_SECOND)

class ObstacleAvoider
{
	RunningParameters& params;
	int savedTurningTime,
		savedSlopeMovementTime,
		savedForwardTime;
	int savedSpeed;

	static double ToSeconds(int timeInMs)
	{
		return timeInMs / MS_IN_SECOND_D;
	}

	static int ToMilliseconds(double timeInSec)
	{
		return static_cast<int>(MS_IN_SECOND * timeInSec);
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
		if (params.GetValue<int>(RobotParameters::speed) != savedSpeed)
		{
			savedSpeed = params.GetValue<int>(RobotParameters::speed);
			double actualSpeedInMm = savedSpeed * params.GetValue<int>(RobotParameters::speedUnitInMm);

			// note there is always safe turning angle of 90 deg
			// we need to move at least half chassis + wheel + some safe distance to obstacle
			double minXDisplacement = params.GetValue<double>(RobotParameters::OBSTACLE_DISTANCE) + 
				(params.GetValue<double>(RobotParameters::CHASSIS_WIDTH) + params.GetValue<double>(RobotParameters::WHEEL_WIDTH)) / 2;

			// those are interesting distances to the obstacle
			double maxYDisplacement = params.GetValue<double>(RobotParameters::FARTHEST_DETECTABLE_RANGE) * actualSpeedInMm;

			// work in 5 degree steps
			double xDisplacement[NUM_BINS], yDisplacement[NUM_BINS];
			double radianStep = M_PI * 5 / 180;
			double spinMultiplierOnTurns = params.GetValue<double>(RobotParameters::spinMultiplierOnTurns);
			double smallTurnRadius = params.GetValue<double>(RobotParameters::CHASSIS_WIDTH) / (spinMultiplierOnTurns - 1);

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
					xDisplacement[i] = (spinMultiplierOnTurns + 1) * smallTurnRadius * (1 - cos(i * radianStep)) + slopeTimeInSeconds * actualSpeedInMm * sin(i * radianStep);
					yDisplacement[i] = (spinMultiplierOnTurns + 1) * smallTurnRadius * sin(i * radianStep) + slopeTimeInSeconds * actualSpeedInMm * cos(i * radianStep);
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
			savedForwardTime = ToMilliseconds((params.GetValue<double>(RobotParameters::OBSTACLE_LENGTH) + maxYDisplacement - yDisplacement[goodAlternative]) / actualSpeedInMm);
		}

		turningTime = savedTurningTime;
		slopeMovementTime = savedSlopeMovementTime;
		forwardTime = savedForwardTime;
	}
};