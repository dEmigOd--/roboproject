#pragma once

#include <memory>
#include <vector>

typedef std::shared_ptr<class Serial> serial_ptr;

class Serial
{
public:
	typedef unsigned char byte;
	typedef std::vector<byte> byte_vec;

	virtual ~Serial()
	{
	}
	virtual bool connect() = 0;
	virtual void disconnect() = 0;
	virtual bool available() = 0;
	virtual int  read(unsigned timeout, byte* buffer, int len) = 0;
	virtual bool read(unsigned timeout, byte& data) = 0;
	virtual void write(const byte* buffer, unsigned len) = 0;

	static serial_ptr create(const char* port, unsigned speed);
};


template<class T>
inline Serial::byte B(const T& t)
{
	return Serial::byte(t & 0xFF);
}
