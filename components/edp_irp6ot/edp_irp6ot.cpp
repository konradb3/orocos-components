/*
 * edp_irp6ot.cpp
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

#include "edp_irp6ot.h"

namespace orocos_test
{
edp_irp6ot::edp_irp6ot(std::string name) :
	TaskContext(name), positionSetpoint_port("Position_setpoint"),
			positionCurrent_port("Current_position"), number_of_axes(
					"IRP6OT_NUM_AXES", 7)
{
	this->ports()->addPort(&positionSetpoint_port);
	this->ports()->addPort(&positionCurrent_port);

	this->attributes()->addConstant(&number_of_axes);

}

bool edp_irp6ot::configureHook()
{
	return true;
}

bool edp_irp6ot::startHook()
{
	joint_pos.resize(7);
	joint_pos[0] = 0.0;
	joint_pos[1] = 0.101;
	joint_pos[2] = 1.50;
	joint_pos[3] = -0.2;
	joint_pos[4] = -0.8;
	joint_pos[5] = -2.0;
	joint_pos[6] = 1.0;

	return true;
}

void edp_irp6ot::updateHook()
{

	if (joint_pos.size() == 7)
		positionCurrent_port.Set(joint_pos);

	positionSetpoint_port.Get(joint_pos);
	//log(RTT::Error) << "test" << RTT::endlog();
}

void edp_irp6ot::stopHook()
{

}

void edp_irp6ot::cleanupHook()
{

}
}
ORO_CREATE_COMPONENT( orocos_test::edp_irp6ot )
;
