/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	SerialPort class
//	implements linux low-level serial io port device
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/


#ifdef __linux__
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <termios.h>

#include <cstring>
#include <cerrno>

#include <string>
#include <stdexcept>
#include <vector>

#include "SerialPort.h"


el::Logger* SerialPort::logger = el::Loggers::getLogger("ttyUSB0");

void SerialPort::SetInterfaceConfigurations(BaudRate speed, CharacterWidth width, Parity parity, NumStopBits stopb, FlowControl ctrl)
{
	struct termios tty;
	std::memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		throw std::runtime_error("error " + std::to_string(errno) + " from tcgetattr");
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
													// disable IGNBRK for mismatched speed tests; otherwise receive break
													// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag |= stopb;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		throw std::runtime_error("error " + std::to_string(errno) + " from tcsetattr");
	}
}

void SerialPort::Open(BaudRate speed, CharacterWidth width, Parity parity, NumStopBits stopb, FlowControl ctrl)
{
	fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		fd = PORT_CLOSED;
		throw std::runtime_error("error " + std::to_string(errno) + " openning " + portname + ": " + strerror(errno));
	}

	SetInterfaceConfigurations(speed, width, parity, stopb, ctrl);
}

void SerialPort::Close()
{
	if (fd != PORT_CLOSED)
	{
		close(fd);
	}
}

bool SerialPort::IsDataAvailable() const
{
	throw std::logic_error("Not implemented");
}

void SerialPort::Read(DataBuffer buffer, int length, unsigned timeout)
{
	throw std::logic_error("Not implemented");
}

unsigned char SerialPort::ReadByte(unsigned timeout)
{
	throw std::logic_error("Not implemented");
}

void SerialPort::Write(DataBuffer buffer)
{
	logger->verbose(2, "Port got %v.", std::string(buffer.begin(), buffer.end()));
	int nwritten = write(fd, buffer.data(), buffer.size());
	if (nwritten != (int) buffer.size())
	{
		throw std::runtime_error("error " + std::to_string(errno) + " writting to serial " + portname + ": " + strerror(errno));
	}
}

#endif
