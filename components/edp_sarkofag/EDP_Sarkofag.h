/*
 * Sarkofag.h
 *
 *  Created on: 2010-04-08
 *      Author: konradb3
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

#ifndef SARKOFAG_H_
#define SARKOFAG_H_

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Properties.hpp>

namespace orocos_test
{

class EDP_Sarkofag: public RTT::TaskContext
{
public:
	EDP_Sarkofag(std::string name);
	/**
	 * This function is for the configuration code.
	 * Return false to abort configuration.
	 */
	bool configureHook();
	/**
	 * This function is for the application's start up code.
	 * Return false to abort start up.
	 */
	bool startHook();

	/**
	 * This function is called by the Execution Engine.
	 */
	void updateHook();

	/**
	 * This function is called when the task is stopped.
	 */
	void stopHook();

	/**
	 * This function is called when the task is being deconfigured.
	 */
	void cleanupHook();
protected:
	RTT::DataPort<double> positionSetpoint_port;
	RTT::DataPort<double> positionCurrent_port;

	RTT::Property<std::string> dev_prop;
private:
	int fd;
    struct termios oldtio;

    char data[10];
};

}

#endif /* SARKOFAG_H_ */
