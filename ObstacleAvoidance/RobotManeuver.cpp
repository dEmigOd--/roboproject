#include "RobotManeuver.h"

el::Logger* RobotManeuver::creatorLogger = el::Loggers::getLogger("Maneuver-ADD");
el::Logger* RobotManeuver::workerLogger = el::Loggers::getLogger("Maneuver-EXE");

bool inManeuver = false;

void StartManeuver()
{
	inManeuver = true;
}

void FinishManeuver()
{
	inManeuver = false;
}

void RobotManeuver::CommandLoop()
{
	while (active)
	{
		// wait until there is work to do
		std::unique_lock<std::mutex> locker(commandPoolLock);
		waitForWork.wait(locker, [this]
		{
			return !commands.empty();
		});

		// get next command and let mutex go
		Cmd nextCommand = commands.front();
		commands.pop();

		locker.release();
		commandPoolLock.unlock();

		// execute next command
		workerLogger->verbose(2, "Executing %v command", nextCommand.Name());
		nextCommand();
	}

	robot.Stop();
}

void RobotManeuver::AddCommand(Cmd cmd, bool exclusive)
{
	if(needToBeLocked)
	{
		std::lock_guard<std::mutex> locker(commandPoolLock);

		if (exclusive)
		{
			FinishManeuver();
			commands = std::queue<Cmd>();
		}

		creatorLogger->verbose(2, "Adding %v command", cmd.Name());
		commands.push(cmd);
	}
	// notify the queue
	waitForWork.notify_one();
}

void RobotManeuver::Separator(CmdPtr cmd)
{
	AddCommand(Cmd("FINISH MANEUVER", cmd));
}

void RobotManeuver::CoolDown(int magicMilis)
{
	AddCommand(Cmd("CONTINUE-" + to_string(magicMilis), CmdPtr(
		[this, magicMilis]() -> void
		{
			workerLogger->verbose(2, "Started waiting");
			auto start = std::chrono::high_resolution_clock::now();

			std::this_thread::sleep_for(std::chrono::milliseconds(magicMilis));

			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double, std::milli> elapsed = end - start;

			workerLogger->verbose(2, "Waited %v ms", elapsed.count());
	})));
}

void RobotManeuver::Maneuver(CmdFunc firstFunc, CmdFunc secondFunc)
{
	if (!inManeuver) 
	{
		Separator(StartManeuver);

		(this->*firstFunc)();
		CoolDown(200);

		Forward();
		CoolDown(380);

		(this->*secondFunc)();
		CoolDown(400);

		Forward();
		CoolDown(380);

		(this->*firstFunc)();
		CoolDown(200);

		Forward();

		Separator(FinishManeuver);
	}
}

void RobotManeuver::Maneuver2(CmdFunc firstFunc, CmdFunc secondFunc)
{
	// we suppose both passed functinos are left/right turns in any order
	int turningTime, slopeMovementTime, forwardTime;
	
	navigator.DetectSafeTurningParameters(turningTime, slopeMovementTime, forwardTime);

	if (!inManeuver)
	{
		Separator(StartManeuver);

		(this->*firstFunc)();
		CoolDown(turningTime);

		Forward();
		CoolDown(slopeMovementTime);

		(this->*secondFunc)();
		CoolDown(turningTime);

		Forward();
		CoolDown(forwardTime);

		(this->*secondFunc)();
		CoolDown(turningTime);

		Forward();
		CoolDown(slopeMovementTime);

		(this->*firstFunc)();
		CoolDown(turningTime);

		Forward();

		Separator(FinishManeuver);
	}
}

RobotManeuver::RobotManeuver(RunningParameters& params)
	:robot(RobotController(params)), navigator(params), params(params), worker(&RobotManeuver::CommandLoop, this)
{
}

RobotManeuver::~RobotManeuver()
{
	active = false;
	// fake command in case pool is empty
	AddCommand(Cmd("DESTROY", [] () -> void{}), true);
	// wait for pool to stop
	worker.join();
}

void RobotManeuver::Forward()
{
	AddCommand(Cmd("FORWARD", CmdPtr([this]() -> void
	{
		robot.Forward();
	})));
}

void RobotManeuver::Backward()
{
	AddCommand(Cmd("BACKWARD", CmdPtr([this]() -> void
	{
		robot.Backward();
	})));
}

void RobotManeuver::Stop()
{
	AddCommand(Cmd("STOP", CmdPtr([this]() -> void
	{
		robot.Stop();
	})));
}

void RobotManeuver::ImmediateStop()
{
	AddCommand(Cmd("IMMEDIATE STOP", CmdPtr([this]() -> void
	{
		robot.Stop();
	})), true);
}

void RobotManeuver::Right()
{
	AddCommand(Cmd("RIGHT", CmdPtr([this]() -> void
	{
		robot.Right(RobotController::TURBO);
	})));
}

void RobotManeuver::Left()
{
	AddCommand(Cmd("LEFT", CmdPtr([this]() -> void
	{
		robot.Left(RobotController::TURBO);
	})));
}

void RobotManeuver::SmoothRight()
{
	AddCommand(Cmd("SMOOTH RIGHT", CmdPtr([this]() -> void
	{
		robot.Right(RobotController::REGULAR);
	})));
}

void RobotManeuver::SmoothLeft()
{
	AddCommand(Cmd("SMOOTH LEFT", CmdPtr([this]() -> void
	{
		robot.Left(RobotController::REGULAR);
	})));
}

void RobotManeuver::ManeuverRight()
{
	Maneuver2((CmdFunc)&RobotManeuver::SmoothRight, (CmdFunc)&RobotManeuver::SmoothLeft);
}

void RobotManeuver::ManeuverLeft()
{
	Maneuver2((CmdFunc)&RobotManeuver::SmoothLeft, (CmdFunc)&RobotManeuver::SmoothRight);
}

void RobotManeuver::Turn90Deg(DirectionEnum dir)
{
	Stop();
	if (dir == RIGHT)
	{
		Right();
	}
	else
	{
		Left();
	}
	CoolDown(400);
	Stop();
}

void RobotManeuver::ForwardStep(double factor)
{
	Forward();
	CoolDown(350);
	Stop();
}

