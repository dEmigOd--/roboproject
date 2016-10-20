#pragma once

#include <string>
#include <stdexcept>
#include <vector>
#include "Logger.h"

#ifdef __linux__
#include <termios.h>
#endif

class SerialPort
{
private:
	static el::Logger* logger;
	static const int PORT_CLOSED = -1;

	std::string portname;
	int fd = PORT_CLOSED;

public:
	typedef std::vector<unsigned char> DataBuffer;

	enum BaudRate
	{
#ifdef __linux__
		BAUD_9600 = B9600,
		BAUD_19200	= B19200,
		BAUD_38400	= B38400,
		BAUD_57600	= B57600,
		BAUD_115200	= B115200,
#else	
#undef		BAUD_9600
#undef		BAUD_19200
#undef		BAUD_38400
#undef		BAUD_57600
#undef		BAUD_115200
		BAUD_9600,
		BAUD_19200,
		BAUD_38400,
		BAUD_57600,
		BAUD_115200,
#endif
		BAUD_DEFAULT = BAUD_115200,
	};
	enum CharacterWidth
	{
#ifdef __linux__
		CHAR_SIZE_8	= CS8,
#else
		CHAR_SIZE_8,
#endif
	};
	enum Parity
	{
#ifndef __linux__
#undef		PARITY_NONE
#endif
		PARITY_NONE
	};
	enum NumStopBits
	{
#ifdef __linux__
		STOP_BITS_1,
		STOP_BITS_2	= CSTOPB,
#else
		STOP_BITS_1,
		STOP_BITS_2,
#endif
	};
	enum FlowControl
	{
		FLOW_CONTROL_NONE
	};

	SerialPort(const std::string& portname)
		: portname(portname)
	{
	}

	~SerialPort()
	{
		Close();
	}

	void Open(BaudRate speed, CharacterWidth width, Parity parity, NumStopBits stopb, FlowControl ctrl);

	void Close();

	bool IsDataAvailable() const;

	void Read(DataBuffer buffer, int length, unsigned timeout);

	unsigned char ReadByte(unsigned timeout);

	void Write(DataBuffer buffer);

private:
	void SetInterfaceConfigurations(BaudRate speed, CharacterWidth width, Parity parity, NumStopBits stopb, FlowControl ctrl);
};
