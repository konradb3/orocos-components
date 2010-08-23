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
	cloud_port("PointCloud_input")
{
	this->ports()->addPort(cloud_port);
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



	if (cloud_port.read(cloud) == RTT::NewData)
	{
	cloud_proxy.clear_vertices();
    for(int y = 0; y < cloud.rows; y++)
    {
        for(int x = 0; x < cloud.cols; x++)
        {
            cv::Vec3f point = cloud.at<cv::Vec3f>(y, x);
            if(fabs(point[2] - 2000) < FLT_EPSILON || fabs(point[2]) > 2000) continue;
            set.add_vertex(point[0], point[1], point[2]);
        }
    }

	cloud_proxy.add_vertices(set);
	}
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
