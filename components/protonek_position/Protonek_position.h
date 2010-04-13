/*
 * Protonek_position.h
 *
 *  Created on: Dec 28, 2009
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

#ifndef PROTONEK_POSITION_H_
#define PROTONEK_POSITION_H_

#include <vector>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Properties.hpp>

#include <kdl/frames.hpp>

#include "protonek.h"

namespace orocos_test
{

class Protonek_position: public RTT::TaskContext
{
public:
	Protonek_position(std::string name);

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
	RTT::DataPort<KDL::Twist> velocitySetpoint_port;
	RTT::DataPort<KDL::Twist> velocity_port;
	RTT::DataPort<KDL::Frame> position_port;
	RTT::Property<std::string> port_prop;
private:

	Protonek protonek;
	std::string port_name;

	KDL::Twist velocitySetpoint;
	KDL::Twist velocity;
	KDL::Frame position;
};

}

#endif /* PROTONEK_POSITION_H_ */
