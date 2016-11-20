/*M/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	PosixSerial class
//	implements posix serial io port device
//
//	only on linux
//
//	Author: Dmitry Rabinovich
//	Copyright (C) 2016 Technion, IIT
//
//	2016, November 19
//
//M*/


#ifdef __linux__

#include "serial.h"
#include "SerialPort.h"

class PosixSerial : public Serial
{
	SerialPort::BaudRate      m_Speed;
	SerialPort                m_Port;
public:
	PosixSerial(const char* port, unsigned speed)
		: m_Port(port)
	{
		switch (speed)
		{
			case 9600:   m_Speed = SerialPort::BAUD_9600;   break;
			case 19200:  m_Speed = SerialPort::BAUD_19200;  break;
			case 38400:  m_Speed = SerialPort::BAUD_38400;  break;
			case 57600:  m_Speed = SerialPort::BAUD_57600;  break;
			case 115200: m_Speed = SerialPort::BAUD_115200; break;
			default:     m_Speed = SerialPort::BAUD_DEFAULT;
		}
	}

	virtual bool connect()
	{
		try
		{
			m_Port.Open(m_Speed, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_NONE);
			return true;
		}
		catch (...)
		{
			return false;
		}
	}

	virtual void disconnect()
	{
		try
		{
			m_Port.Close();
		}
		catch (...)
		{
		}
	}

	virtual bool available()
	{
		try
		{
			return m_Port.IsDataAvailable();
		}
		catch (...)
		{
			return false;
		}
	}

	virtual bool read(unsigned timeout, byte& data)
	{
		try
		{
			data = m_Port.ReadByte(timeout);
			return true;
		}
		catch (...)
		{
			return false;
		}
	}

	int read(unsigned timeout, byte* buffer, int len) override
	{
		try
		{
			SerialPort::DataBuffer dbuf(len);
			m_Port.Read(dbuf, len, timeout);
			std::copy(dbuf.begin(), dbuf.end(), buffer);
			return len;
		}
		catch (...)
		{
			return 0;
		}
	}

	virtual void write(const byte* buffer, unsigned len)
	{
		try
		{
			SerialPort::DataBuffer db(buffer, buffer + len);
			m_Port.Write(db);
		}
		catch (...)
		{
		}
	}
};

serial_ptr Serial::create(const char* port, unsigned speed)
{
	return serial_ptr(new PosixSerial(port, speed));
}

#endif