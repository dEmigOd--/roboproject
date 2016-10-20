#pragma once

#include "pwm.h"
#include "Logger.h"
#include "Common.h"
#include <iostream>
#include <map>

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
