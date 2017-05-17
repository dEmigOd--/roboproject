#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotController class
//	implements real abstraction layer to execute actual movement of the robot, without notion of wheels
//
//	inhereted from pervious project
//
//	Author: ben, Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 17
//
//M*/

#include <iostream>
#include <map>

#include "Logger.h"
#include "Common.h"
#include "pwm.h"

class RobotController
{
public:
	enum TurningAngle
	{
		REGULAR,
		SHARP,
		TURBO,
	};

	enum WheelSide
	{
		LEFT_WHEELS,
		RIGHT_WHEELS
	};

private:
	const int FREQ = 1000;
	const bool DISABLE = false;
	const bool ENABLE = true;
	const double SLOWING_RATE = 4.0;
	const int FULL_STOP = 0;

	PWM::pwm_ptr p;
	RunningParameters& params;

	void SetNewWheelSpeed(int leftSpeed, int rightSpeed);
	int GetNewTurningSpeed(TurningAngle value);

	static el::Logger* logger;

	int Speed() const;
public:
	RobotController(RunningParameters& params);
	~RobotController();

	void SetSpeed(int newSpeed);
	void Forward();
	void Backward();
	void Stop();
	void Right(TurningAngle value);
	void Left(TurningAngle value);

};
