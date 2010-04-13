/*
 * Vis_peekabot.cpp
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

#include "Vis_peekabot.h"

namespace orocos_test
{

Vis_peekabot::Vis_peekabot(std::string name) :
	TaskContext(name),
	cloud_port("Point_Cloud_input")
{
	this->ports()->addEventPort(&cloud_port);
}
bool Vis_peekabot::configureHook()
{
	return true;
}

bool Vis_peekabot::startHook()
{
	client.connect("192.168.18.11");
	cloud_proxy.add(client, "root.cloud", peekabot::REPLACE_ON_CONFLICT);
	return true;
}

void Vis_peekabot::updateHook()
{
	
	peekabot::VertexSet set;
	static int size = 0;

	if(size > 120)
	{
		size =0;	
		cloud_proxy.clear_vertices();
	}
	cloud_port.Get(cloud);

	for (unsigned int i = 0; i<cloud.size(); i++)
		set.add_vertex(cloud[i].x(), cloud[i].y(), cloud[i].z());
	cloud_proxy.add_vertices(set);
	++size;
}

void Vis_peekabot::stopHook()
{
	client.disconnect();
}

void Vis_peekabot::cleanupHook()
{

}

}
ORO_CREATE_COMPONENT( orocos_test::Vis_peekabot )
