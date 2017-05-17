/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	PWM classes
//	implements pwm-based commands to actually different (non-PWM) hardware
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016-2017 Technion, IIT
//
//	2017, May 17
//
//M*/

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include "pwm.h"
#include "xstring.h"
#include "Logger.h"
#include "motors.h"

class LoggingPWM : public PWM
{
	static el::Logger* logger;

public:

	virtual void enable(int id, bool state) override
	{
		logger->verbose(2, "enable%v = %v", id, state);
	}

	virtual void set_freq(int id, int freq) override
	{
		logger->verbose(2, "freq%v = %v", id, freq);
	}

	virtual void set_duty_cycle(int id, int value) override
	{
		logger->verbose(2, "duty%v = %v", id, value);
	}
};

el::Logger* LoggingPWM::logger = el::Loggers::getLogger("PWM");

class FSPWM : public PWM
{
private:
	static el::Logger* logger;

	static void write_file(const xstring& name, int value)
	{
		static const xstring dir = "/sys/devices/platform/pwm-ctrl/";
		std::ofstream f(dir + name);
		f << value << std::endl;
	}
public:
	FSPWM()
	{
		if (system("modprobe pwm-meson npwm=2"))
		{
			logger->verbose(2, "Succeded in starting pwm-meson");
		}
		if (system("modprobe pwm-ctrl"))
		{
			logger->verbose(2, "Succeded in starting pwm-ctrl");
		}
	}

	virtual void enable(int id, bool state) override
	{
		write_file("enable" + xstring(id), state ? 1 : 0);
	}

	virtual void set_freq(int id, int freq) override
	{
		write_file("freq" + xstring(id), freq);
	}

	virtual void set_duty_cycle(int id, int value) override
	{
		write_file("duty" + xstring(id), value);
	}


	virtual bool CouldBeStopped() const
	{
		return true;
	}
};

el::Logger* FSPWM::logger = el::Loggers::getLogger("OdroidPWM");


class ArduinoPWM : public PWM
{
private:
	static el::Logger* logger;

	RobotMotors motors;
	int lastSeenLeftSpeed = 0;
	int lastSeenRightSpeed = 0;
public:
	virtual void enable(int id, bool state) override
	{
		if (state == 0)
		{
			motors.stop();
		}
	}

	virtual void set_freq(int id, int freq) override
	{
		// no meaning for me
	}

	virtual void set_duty_cycle(int id, int value) override
	{
		(id == 1 ? lastSeenRightSpeed : lastSeenLeftSpeed) = value;

		motors.drive(lastSeenLeftSpeed, lastSeenRightSpeed);
	}

	virtual bool CouldBeStopped() const
	{
		return true;
	}
};

el::Logger* ArduinoPWM::logger = el::Loggers::getLogger("ArduinoPWM");

class WrappingPWM : public PWM
{
private:
	std::vector<PWM::pwm_ptr> loggers;
	const RunningParameters& params;

	bool ShouldRun(PWM::pwm_ptr ptr)const
	{
		return (params.GetValue<bool>(RobotParameters::shouldMove) || !ptr->CouldBeStopped());
	}
public:
	WrappingPWM(const RunningParameters& params)
		: params(params)
	{
		loggers = std::vector<PWM::pwm_ptr>();
		loggers.push_back(PWM::pwm_ptr(new LoggingPWM()));
#ifdef __linux__ 
		// loggers.push_back(PWM::pwm_ptr(new FSPWM()));
		loggers.push_back(PWM::pwm_ptr(new ArduinoPWM()));
#endif
	}

	virtual void enable(int id, bool state) override
	{
		for (auto logger : loggers)
		{
			if (ShouldRun(logger))
			{
				logger->enable(id, state);
			}
		}
	}

	virtual void set_freq(int id, int freq) override
	{
		for (auto logger : loggers)
		{
			if (ShouldRun(logger))
			{
				logger->set_freq(id, freq);
			}
		}
	}

	virtual void set_duty_cycle(int id, int value) override
	{
		for (auto logger : loggers)
		{
			if (ShouldRun(logger))
			{
				logger->set_duty_cycle(id, value);
			}
		}
	}
};

PWM::pwm_ptr PWM::create(const RunningParameters& params)
{
	return PWM::pwm_ptr(new WrappingPWM(params));
}
