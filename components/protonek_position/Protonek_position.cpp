/*
 * Protonek_position.cpp
 *
 *  Created on: Dec 28, 2009
 *      Author: konrad
 */
#include <ocl/ComponentLoader.hpp>

#include "Protonek_position.h"

namespace orocos_test
{

Protonek_position::Protonek_position(std::string name) :
	TaskContext(name, PreOperational),
	velocitySetpoint_port("VelocitySetpoint_input"),
	velocity_port("Velocity_output"),
	position_port("Position_output"),
	port_prop("Device", "UNIX device file (/dev/ttySx)")
{
	this->ports()->addPort(&velocitySetpoint_port);
	this->ports()->addPort(&position_port);

	this->properties()->addProperty(&port_prop);

}

bool Protonek_position::configureHook()
{
	port_name = port_prop;
	return true;
}

bool Protonek_position::startHook()
{
	if(protonek.connect(port_name.c_str(), 9600) == false)
	{
		log(RTT::Fatal) << "Unable to connect to protonek driver : " << port_name << RTT::endlog();
		return false;
	}

	return true;
}

void Protonek_position::updateHook()
{
	double rot;

	velocitySetpoint_port.Get(velocitySetpoint);
	protonek.setVelocity(velocitySetpoint.vel.x(), velocitySetpoint.rot.z());
	protonek.update();
	protonek.updateOdometry();
	protonek.getVelocity(velocity.vel.data[0],velocity.rot.data[2]);
	protonek.getOdometry(position.p.data[0],position.p.data[1], rot);
	position.M = KDL::Rotation::RPY(0.0, 0.0, rot);

	velocity_port.Set(velocity);
	position_port.Set(position);
}

void Protonek_position::stopHook()
{
	protonek.disconnect();
}

void Protonek_position::cleanupHook()
{

}

}
ORO_CREATE_COMPONENT( orocos_test::Protonek_position );
