/*
 * Laser_tilt.h
 *
 *  Created on: Jan 8, 2010
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

#ifndef LASER_TILT_H_
#define LASER_TILT_H_

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Properties.hpp>
#include <rtt/Command.hpp>

#include "ourptz.h"

namespace orocos_test
{
class Laser_tilt: public RTT::TaskContext
{
public:
	Laser_tilt(std::string name);
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
	RTT::DataPort<double> position_port;
	RTT::Command<bool(double)> move;
	RTT::Property<std::string> port_prop;
private:
	Ourptz *ptz;
	double target;
	double position;

	bool move_impl(double d);
	bool atpos_impl(double d);
};

}

#endif /* LASER_TILT_H_ */
