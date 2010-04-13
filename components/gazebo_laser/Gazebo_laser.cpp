/*
 * Gazebo_laser.cpp
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

#include "Gazebo_laser.h"

namespace orocos_test
{

Gazebo_laser::Gazebo_laser(std::string name):
	TaskContext(name,PreOperational),
    distances_port("LaserDistance"),
    angles_port("LaserAngle"),
    laserIface_prop("Laser_Interface", "Gazebo laser interface name")
{
	this->ports()->addPort(&distances_port);
	this->ports()->addPort(&angles_port);

	this->properties()->addProperty(&laserIface_prop);
}

bool Gazebo_laser::configureHook()
{
	  client = new gazebo::Client();
	  laserIface = new gazebo::LaserIface();

	  laserIface_name = laserIface_prop;
	return true;
}

bool Gazebo_laser::startHook()
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
	    laserIface->Open(client, laserIface_name); // "protonek_model::position_iface_0"
	  }
	  catch (std::string e)
	  {
		  log(RTT::Error) << "Gazebo error: Unable to connect to the laser interface\n"
	    << e << RTT::endlog();
	    return false;
	  }

	  mens_count = laserIface->data->range_count;

	  distances_local.resize(mens_count, 0.0);
	  angles_local.resize(mens_count,0.0);

	  for(int i = 0;i<mens_count;i++)
		  angles_local[i] = laserIface->data->min_angle + (double)i * laserIface->data->res_angle;

	  angles_port.Set(angles_local);

	  return true;
}

void Gazebo_laser::updateHook()
{

	for(int i = 0; i<mens_count; i++)
		distances_local[i] = laserIface->data->ranges[i];
	distances_port.Set(distances_local);

}

void Gazebo_laser::stopHook()
{
	laserIface->Close();
	client->Disconnect();
}

void Gazebo_laser::cleanupHook()
{
	delete laserIface;
	delete client;
}

}
ORO_CREATE_COMPONENT( orocos_test::Gazebo_laser );
