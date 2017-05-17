/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotController class
//	implementation
//
//	Author: ben, Dmitry Rabinovich
//	Copyright (C) 201-2017 Technion, IIT
//
//	2017, May 17
//
//M*/

#include "RobotController.h"
#include "Utils.h"

el::Logger* RobotController::logger = el::Loggers::getLogger("Controller");

RobotController::RobotController(RunningParameters& params)
	: params(params)
{
	p = PWM::create(params);
}

int RobotController::Speed() const
{
	return params.GetValue<int>(RobotParameters::speed);
}

void RobotController::Forward()
{
	logger->verbose(2, "FORWARD command issued");
	SetNewWheelSpeed(Speed(), Speed());
}

void RobotController::Backward()
{
	logger->verbose(2, "BACKWARD command issued");
	SetNewWheelSpeed(-Speed(), -Speed());
}

void RobotController::Stop()
{
	logger->verbose(2, "STOP command issued");
	p->enable(RobotController::RIGHT_WHEELS, DISABLE);
	p->enable(RobotController::LEFT_WHEELS, DISABLE);
}

RobotController::~RobotController()
{
	Stop();
}

int RobotController::GetNewTurningSpeed(TurningAngle angle)
{
	int newSpeed = RobotController::FULL_STOP;
	switch (angle)
	{
		case RobotController::REGULAR:
			newSpeed = (int)(Speed() / params.GetValue<double>(RobotParameters::spinMultiplierOnTurns));
			break;
		case RobotController::TURBO:
			newSpeed = -Speed();
		default:
			break;
	}
	return newSpeed;
}

void RobotController::Right(TurningAngle angle)
{
	int rightSpeed = GetNewTurningSpeed(angle);
	logger->verbose(2, "TURN RIGHT %v command issued", angle);
	SetNewWheelSpeed(Speed(), rightSpeed);
}

void RobotController::Left(TurningAngle angle)
{
	int leftSpeed = GetNewTurningSpeed(angle);
	logger->verbose(2, "TURN LEFT %v command issued", angle);
	SetNewWheelSpeed(leftSpeed, Speed());
}

void RobotController::SetNewWheelSpeed(int leftSpeed, int rightSpeed)
{
	p->enable(RobotController::LEFT_WHEELS, ENABLE);
	p->enable(RobotController::RIGHT_WHEELS, ENABLE);

	//p->set_freq(RobotController::LEFT_WHEELS, FREQ);
	//p->set_freq(RobotController::RIGHT_WHEELS, FREQ);

	p->set_duty_cycle(RobotController::LEFT_WHEELS, leftSpeed);
	p->set_duty_cycle(RobotController::RIGHT_WHEELS, rightSpeed);
}

void RobotController::SetSpeed(int newSpeed)
{
	logger->verbose(2, "SET SPEED %v command issued", newSpeed);
	params.rip.SetValue(RobotParameters::speed, CastTo<std::string>(newSpeed));
}

