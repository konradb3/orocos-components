/*
 * Gazebo_position.cpp
 *
 *  Created on: Dec 27, 2009
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

#include "Gazebo_position.h"

namespace orocos_test
{

Gazebo_position::Gazebo_position(std::string name) :
	TaskContext(name, PreOperational),
	velocitySetpoint_port("VelocitySetpoint_input"),
	velocity_port("Velocity_output"),
	position_port("Position_output"),
	posIface_prop("Position_Interface", "Gazebo position interface name")
{
	this->ports()->addPort(&velocitySetpoint_port);
	this->ports()->addPort(&position_port);

	this->properties()->addProperty(&posIface_prop);

}

bool Gazebo_position::configureHook()
{
	  client = new gazebo::Client();
	  simIface = new gazebo::SimulationIface();
	  posIface = new gazebo::PositionIface();

	  posIface_name = posIface_prop;
	return true;
}

bool Gazebo_position::startHook()
{
	  // Connect to the libgazebo server
	  try
	  {
	    client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
	  }
	  catch (std::string e)
	  {
	    log(RTT::Error) << "Unable to connect\n" << e << RTT::endlog();
	    return false;
	  }

	  /// Open the Position interface
	  try
	  {
	    posIface->Open(client, posIface_name); // "protonek_model::position_iface_0"
	  }
	  catch (std::string e)
	  {
		  log(RTT::Error) << "Gazebo error: Unable to connect to the position interface\n"
	    << e << RTT::endlog();
	    return false;
	  }
	  // Enable the motor
	  posIface->Lock(1);
	  posIface->data->cmdEnableMotors = 1;
	  posIface->Unlock();
	return true;
}

void Gazebo_position::updateHook()
{
	velocitySetpoint_port.Get(velocitySetpoint);

	posIface->Lock(1);

	posIface->data->cmdVelocity.pos.x = velocitySetpoint.vel.x();
	posIface->data->cmdVelocity.pos.y = velocitySetpoint.vel.y();
	posIface->data->cmdVelocity.pos.z = velocitySetpoint.vel.z();

	posIface->data->cmdVelocity.roll = velocitySetpoint.rot.x();
	posIface->data->cmdVelocity.pitch = velocitySetpoint.rot.y();
	posIface->data->cmdVelocity.yaw = velocitySetpoint.rot.z();

	position = KDL::Frame(KDL::Rotation::RPY(posIface->data->pose.roll,
									  posIface->data->pose.pitch,
									  posIface->data->pose.yaw),
				   KDL::Vector(posIface->data->pose.pos.x,
						       posIface->data->pose.pos.y,
						       posIface->data->pose.pos.z));

	velocity.vel.x(posIface->data->velocity.pos.x);
	velocity.vel.y(posIface->data->velocity.pos.y);
	velocity.vel.z(posIface->data->velocity.pos.z);

	velocity.rot.x(posIface->data->velocity.roll);
	velocity.rot.y(posIface->data->velocity.pitch);
	velocity.rot.z(posIface->data->velocity.yaw);

	posIface->Unlock();

	position_port.Set(position);
	velocity_port.Set(velocity);

}

void Gazebo_position::stopHook()
{
	posIface->Close();
	client->Disconnect();
}

void Gazebo_position::cleanupHook()
{
	delete posIface;
	delete client;
}

}
ORO_CREATE_COMPONENT( orocos_test::Gazebo_position );
