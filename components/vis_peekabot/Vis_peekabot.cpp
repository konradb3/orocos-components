/*
 * Vis_peekabot.cpp
 *
 *  Created on: Jan 9, 2010
 *      Author: konrad
 */

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
