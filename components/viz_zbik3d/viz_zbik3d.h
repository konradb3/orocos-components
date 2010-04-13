/*
 * viz_zbik3d.h
 *
 *  Created on: Jan 9, 2010
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


#ifndef VIZ_ZBIK3D_HPP
#define VIZ_ZBIK3d_HPP

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <vector>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Properties.hpp>

#define MAX_SERVOS_NR 8
#define MAXBUFLEN 100

namespace orocos_test
{

class viz_zbik3d: public RTT::TaskContext
{
public:
	viz_zbik3d(std::string name);
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

	RTT::DataPort<std::vector<double> > joint_port;
	RTT::Property<int> port_prop;
private:
	int sockfd;
	struct sockaddr_in my_addr; // my address information
	struct sockaddr_in their_addr; // connector's address information
	socklen_t addr_len;
	char buf[MAXBUFLEN];
	std::vector<double> data;

	struct
	{
		int synchronised;
		float joints[MAX_SERVOS_NR];
	} reply;
};

}
#endif
