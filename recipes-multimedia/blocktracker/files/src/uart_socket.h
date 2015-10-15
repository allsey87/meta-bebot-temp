#ifndef UART_SOCKET_H
#define UART_SOCKET_H

#include <cstdint>
#include <cstddef>

#include <termios.h>

class CUARTSocket {
	/* file descriptor for the serial port */
	int m_nPort;

	struct termios sPrevPortSettings;
public:

	CUARTSocket() : m_nPort(-1) {}

	~CUARTSocket() {
		if(m_nPort != -1) {
			Close();
		}
	}

	/*  open a serial port connection */
	int Open(const char* pch_port, unsigned int un_baud = 115200);

	/* close the connection */
	int Close();

	/* read into buffer */ 
	int16_t Read(uint8_t* p_buffer, size_t un_length) const;

	/* write from buffer */
	int16_t Write(const uint8_t* p_buffer, size_t un_length) const;

};

#endif
