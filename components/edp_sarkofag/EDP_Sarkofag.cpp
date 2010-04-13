/*
 * Sarkofag.cpp
 *
 *  Created on: 2010-04-08
 *      Author: Konrad Banachowicz
 *      Copyright : (C) 2010
 *
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <time.h>

#include <ocl/ComponentLoader.hpp>

#include "EDP_Sarkofag.h"

#define PORT "/dev/ttyMI0"
#define BAUD  B921600

namespace orocos_test
{

EDP_Sarkofag::EDP_Sarkofag(std::string name) :
	TaskContext(name, PreOperational), positionSetpoint_port("Position_setpoint"),
			positionCurrent_port("Position_current"), dev_prop("Port", "UNIX serial port", "/dev/ttyMI0")
{
	this->ports()->addPort(&positionSetpoint_port);
	this->ports()->addPort(&positionCurrent_port);

	this->properties()->addProperty(&dev_prop);
}

bool EDP_Sarkofag::configureHook()
{
	fd = open(dev_prop.get().c_str(), O_RDWR | O_NOCTTY);

	tcgetattr(fd, &oldtio);

	// set up new settings
	struct termios newtio;
	memset(&newtio, 0, sizeof(newtio));
	newtio.c_cflag = CS8 | CLOCAL | CREAD /* | CSTOPB*/;
	newtio.c_iflag = INPCK; //IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
	{
		log(RTT::Error) << "Failed to set serial baud rate" << RTT::endlog();
		tcsetattr(fd, TCSANOW, &oldtio);
		close(fd);
		fd = -1;
		return false;
	}

	// activate new settings
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	return true;
}

bool EDP_Sarkofag::startHook()
{
	return true;
}

void EDP_Sarkofag::updateHook()
{
	int dlen = 0;
	data[0] = 0;
	data[1] = '#';
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;

	write(fd, data, 10);

	fd_set rfds;

	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	// timeout
	struct timeval timeout;
	timeout.tv_sec = (time_t) 0;
	timeout.tv_usec = 500;

	int select_retval = select(fd + 1, &rfds, NULL, NULL, &timeout);

	if (select_retval == 0)
	{
		log(RTT::Warning) << "timeout !!!" << RTT::endlog();
	}
	else
	{
		while (dlen < 8)
		{
			dlen += read(fd, data, 8 - dlen);
		}
	}

	positionCurrent_port.Set((double)((((int16_t*)data)[1])));

//	log(RTT::Debug) << (int)data[0] << " " << (int)data[1] << " " << (int)data[2] << " " << (int)data[3] << " " << (int)data[4] << " " << (int)data[5] << RTT::endlog();

}

void EDP_Sarkofag::stopHook()
{

}

void EDP_Sarkofag::cleanupHook()
{
	/// restore serial settings ///
	if (fd > 0)
		tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
}

}

ORO_CREATE_COMPONENT( orocos_test::EDP_Sarkofag )
;
