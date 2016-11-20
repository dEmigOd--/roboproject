#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	RobotManeuver class
//	implements abstraction layer to execute maneuvering, i.e. around obstacle, not only turn or move
//
//	inhereted from pervious project
//
//	Author: ben, Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/


#include <iostream>
#include <functional>
#include <chrono>
#include <future>
#include <queue>

#include "Common.h"
#include "RobotController.h"
#include "ObstacleAvoider.h"

class RobotManeuver
{
	static el::Logger* creatorLogger;
	static el::Logger* workerLogger;

	typedef std::function<void()> CmdPtr;
	class Cmd
	{
		std::string name;
		CmdPtr cmd;
	public:
		Cmd(const std::string& name, CmdPtr cmd)
			: name(name), cmd(cmd)
		{ }

		const std::string& Name() const
		{
			return name;
		}

		void operator()() const
		{
			cmd();
		}
	};

	RobotController robot;
	ObstacleAvoider navigator;
	RunningParameters& params;

	typedef void (RobotManeuver::*CmdFunc)();

	void Maneuver(CmdFunc firstFunc,CmdFunc secondFunc);
	void Maneuver2(CmdFunc firstFunc,CmdFunc secondFunc);

	void Separator(CmdPtr cmd);
	void CoolDown(int magicMilis);

	bool active = true;
	static const bool needToBeLocked = true;

	std::mutex commandPoolLock;
	std::condition_variable waitForWork;
	std::thread worker;

	std::queue<Cmd> commands;

	void CommandLoop();
	void AddCommand(Cmd cmd, bool exclusive = false);
public:
	enum DirectionEnum{
		RIGHT,
		LEFT
	};
	RobotManeuver(RunningParameters& params);
	~RobotManeuver();
	void Forward();
	void Backward();
	void Stop();
	void ImmediateStop();
	void Right();
	void Left();
	void SmoothRight();
	void SmoothLeft();

	void Turn90Deg(DirectionEnum dir);
	void ForwardStep(double factor = 1);
	void ManeuverRight();
	void ManeuverLeft();
};
