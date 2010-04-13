/*
 * edp_irp6p.cpp
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

#include <ocl/ComponentLoader.hpp>

#include "edp_irp6p.h"

namespace orocos_test
{
edp_irp6p::edp_irp6p(std::string name) :
	TaskContext(name), positionSetpoint_port("Position_setpoint"),
			positionCurrent_port("Current_position"), number_of_axes(
					"IRP6P_NUM_AXES", 6)
{
	this->ports()->addPort(&positionSetpoint_port);
	this->ports()->addPort(&positionCurrent_port);

	this->attributes()->addConstant(&number_of_axes);

}

bool edp_irp6p::configureHook()
{
	return true;
}

bool edp_irp6p::startHook()
{
	joint_pos.resize(6);
	joint_pos[0] = 0.101;
	joint_pos[1] = 1.50;
	joint_pos[2] = -0.2;
	joint_pos[3] = -0.8;
	joint_pos[4] = -2.0;
	joint_pos[5] = 1.0;

	return true;
}

void edp_irp6p::updateHook()
{

	if (joint_pos.size() == 6)
		positionCurrent_port.Set(joint_pos);

	positionSetpoint_port.Get(joint_pos);
	//log(RTT::Error) << "test" << RTT::endlog();
}

void edp_irp6p::stopHook()
{

}

void edp_irp6p::cleanupHook()
{

}
}
ORO_CREATE_COMPONENT( orocos_test::edp_irp6p )
;
