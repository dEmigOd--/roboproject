#pragma once

/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//  IMPORTANT: READ BEFORE USING.
//
//	RobotMotors class
//	implements abstraction layer for robot motors
//
//	*	two motors present [right abd left]
//	*	motors are connected to Arduino actually, which is in turn connected through USB
//	*	so this is platform dependent
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/

#include <thread>

#include "serial.h"
#include "xstring.h"

class RobotMotors
{
	serial_ptr m_Serial;

	void clamp(int& speed)
	{
		if (speed > 127) speed = 127;
		if (speed < -128) speed = -128;
	}

	xstring cmd(xstring side, int speed)
	{
		static const char hex[] = "0123456789ABCDEF";
		side += xstring(1, hex[(speed >> 4) & 15]);
		side += xstring(1, hex[(speed)& 15]);
		return side;
	}

	void write(const xstring& s)
	{
		const char* ptr = s;
		m_Serial->write(reinterpret_cast<const Serial::byte*>(ptr), static_cast<unsigned> (s.length()));
	}
public:
	RobotMotors()
		: m_Serial(Serial::create("/dev/ttyUSB0", 115200))
	{
		if (!m_Serial->connect())
		{
			std::cerr << "Failed to connect to serial port\n";
		}
		else
		{
			std::cout << "Connected!!!\n";
		}
	}

	void drive(int left_speed, int right_speed)
	{
		clamp(left_speed);
		clamp(right_speed);
		// nothing is actually known, but seems, that in current setup A is right speed
		write(cmd("B", left_speed));
		write(cmd("A", right_speed));
	}

	void stop()
	{
		write("S00");
	}

	void turn(int speed)
	{
		drive(speed, -speed);
	}

	void turn(int left_speed, int right_speed)
	{
		drive(left_speed, -right_speed);
	}
};

