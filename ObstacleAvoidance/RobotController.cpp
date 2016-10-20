#include "RobotController.h"
#include "Utils.h"

el::Logger* RobotController::logger = el::Loggers::getLogger("Controller");

RobotController::RobotController(RunningParameters& params)
	: params(params)
{
	p = PWM::create(params);
}

void RobotController::Forward()
{
	logger->verbose(2, "FORWARD command issued");
	SetNewWheelSpeed(params.speed, params.speed);
}

void RobotController::Backward()
{
	logger->verbose(2, "BACKWARD command issued");
	SetNewWheelSpeed(-params.speed, -params.speed);
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
			newSpeed = (int)(params.speed / RobotController::SLOWING_RATE);
			break;
		case RobotController::TURBO:
			newSpeed = -params.speed;
		default:
			break;
	}
	return newSpeed;
}

void RobotController::Right(TurningAngle angle)
{
	int rightSpeed = GetNewTurningSpeed(angle);
	logger->verbose(2, "TURN RIGHT %v command issued", angle);
	SetNewWheelSpeed(params.speed, rightSpeed);
}

void RobotController::Left(TurningAngle angle)
{
	int leftSpeed = GetNewTurningSpeed(angle);
	logger->verbose(2, "TURN LEFT %v command issued", angle);
	SetNewWheelSpeed(leftSpeed, params.speed);
}

void RobotController::SetNewWheelSpeed(int leftSpeed, int rightSpeed)
{
	p->enable(RobotController::LEFT_WHEELS, ENABLE);
	p->enable(RobotController::RIGHT_WHEELS, ENABLE);

	p->set_freq(RobotController::LEFT_WHEELS, FREQ);
	p->set_freq(RobotController::RIGHT_WHEELS, FREQ);

	p->set_duty_cycle(RobotController::LEFT_WHEELS, leftSpeed);
	p->set_duty_cycle(RobotController::RIGHT_WHEELS, rightSpeed);
}

void RobotController::SetSpeed(int newSpeed)
{
	logger->verbose(2, "SET SPEED %v command issued", newSpeed);
	params.speed = newSpeed;
}

