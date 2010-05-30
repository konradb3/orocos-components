/*
 * Protonek_position.cpp
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

#include <ocl/ComponentLoader.hpp>

#include "ProtonekPosition.h"

namespace orocos_test
{

ProtonekPosition::ProtonekPosition(std::string name) :
	TaskContext(name, PreOperational),
	cmdVel_port("cmdVel"),
	msrVel_port("msrVel"),
	msrPos_port("msrPos"),
	port_prop("Device", "UNIX device file (/dev/protonek)")
{
	this->ports()->addPort(&cmdVel_port, "Commanded velocity");
	this->ports()->addPort(&msrVel_port, "Measured velocity");
	this->ports()->addPort(&msrPos_port, "Measured position");

	this->properties()->addProperty(&port_prop);

}

bool ProtonekPosition::configureHook()
{
	port_name = port_prop;
	return true;
}

bool ProtonekPosition::startHook()
{
	if(protonek.connect(port_name.c_str(), 9600) == false)
	{
		log(RTT::Fatal) << "Unable to connect to protonek driver : " << port_name << RTT::endlog();
		return false;
	}

	return true;
}

void ProtonekPosition::updateHook()
{
	double rot;

	cmdVel_port.Get(cmdVel);
	protonek.setVelocity(cmdVel.vel.x(), cmdVel.rot.z());
	protonek.update();
	protonek.updateOdometry();
	protonek.getVelocity(msrVel.vel.data[0],msrVel.rot.data[2]);
	protonek.getOdometry(msrPos.p.data[0],msrPos.p.data[1], rot);
	msrPos.M = KDL::Rotation::RPY(0.0, 0.0, rot);

	msrVel_port.Set(msrVel);
	msrPos_port.Set(msrPos);
}

void ProtonekPosition::stopHook()
{
	protonek.disconnect();
}

void ProtonekPosition::cleanupHook()
{

}

}
ORO_CREATE_COMPONENT( orocos_test::ProtonekPosition );
