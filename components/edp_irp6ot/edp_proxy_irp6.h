/*
 * edp_irp6ot.h
 *
 *  Created on: Dec 11, 2009
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

#ifndef EDP_IRP6P_HPP
#define EDP_IRP6P_HPP

#include <vector>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Properties.hpp>

#include <kdl/frames.hpp>

#include <messip.h>

#include "com_buf.h"


namespace orocos_test
{
class EdpProxyIRP6: public RTT::TaskContext
{
public:
	EdpProxyIRP6(const std::string &name);
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



	RTT::DataPort<std::vector<double> > cmdJntPos_port;
	RTT::DataPort<KDL::Frame> cmdCartPos_port;

	RTT::DataPort<std::vector<double> > msrJntPos_port;
	RTT::DataPort<KDL::Frame> msrCartPos_port;

	RTT::DataPort<KDL::Wrench> cmdWrench_port;
	RTT::DataPort<KDL::Wrench> msrWrench_port;

	RTT::Property<int> control_mode_prop;
	RTT::Property<std::string> mrrocpp_path_prop;
	RTT::Property<std::string> net_attach_point_prop;

	RTT::Property<unsigned int> number_of_axes_prop;
private:
	std::vector<double> cmdJntPos;
	std::vector<double> msrJntPos;

	KDL::Frame cmdCartPos;
	KDL::Frame msrCartPos;

	std::vector<RTT::TaskContext*> peers;

	unsigned int number_of_axes;

	int control_mode;

	mrrocpp::lib::ecp_command_buffer ecp_command;
	mrrocpp::lib::r_buffer reply_package;
	char flkdk[10000];
	messip_channel_t *EDP_fd;
	std::string edp_net_attach_point;

	std::string program_name;
	std::string mrrocpp_path;

	void spawnEDP();
	void send();
	void query();

};
}
#endif
