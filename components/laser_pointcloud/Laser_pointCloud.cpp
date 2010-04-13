/*
 * Laser_pointCloud.cpp
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

#include <ocl/ComponentLoader.hpp>

#include "Laser_pointCloud.h"

namespace orocos_test
{

Laser_pointCloud::Laser_pointCloud(std::string name) :
	TaskContext(name),
	laserDistances_port("Laser_distances_input"),
	laserAngles_port("Laser_angles_input"),
	tiltAngle_port("Tilt_angle_input"),
	cloud_port("Point_Cloud_output")
{
	this->ports()->addEventPort(&laserDistances_port);
	this->ports()->addPort(&laserAngles_port);
	this->ports()->addPort(&tiltAngle_port);
	this->ports()->addPort(&cloud_port);
}

bool Laser_pointCloud::configureHook()
{
	return true;
}

bool Laser_pointCloud::startHook()
{
	laserAngles_port.Get(angles);
	cloud.resize(angles.size(),KDL::Vector::Zero());
	return true;
}

void Laser_pointCloud::updateHook()
{
	laserDistances_port.Get(distances);
	tiltAngle_port.Get(tilt);

	for (unsigned int i = 0; i<distances.size(); i++)
	{
	       // cloud[i] = KDL::Rotation::RotZ(angles[i]) *(KDL::Rotation::RotY(tilt) *  KDL::Vector(0.0,distances[i],0.0))  ;
	//  float X = distance * cos (angle_x) * sin (angle_y);
	 //                             float Y = distance * cos (angle_x) * cos (angle_y);
	 //                             float Z = distance * sin (angle_x);


		cloud[i].z( distances[i] * cos(angles[i] - 1.57) * sin(tilt));
		cloud[i].x( distances[i] * cos(angles[i] - 1.57) * cos(tilt));
		cloud[i].y( distances[i] * sin(angles[i] - 1.57));

	}
	cloud_port.Set(cloud);
}

void Laser_pointCloud::stopHook()
{

}

void Laser_pointCloud::cleanupHook()
{

}

}
ORO_CREATE_COMPONENT( orocos_test::Laser_pointCloud )
