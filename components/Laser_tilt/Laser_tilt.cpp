/*
 * Laser_tilt.cpp
 *
 *  Created on: Jan 8, 2010
 *      Author: konradb3
 */

#include <ocl/ComponentLoader.hpp>

#include "Laser_tilt.h"

namespace orocos_test
{

  Laser_tilt::Laser_tilt(std::string name) :
    TaskContext(name),
    position_port("Position_output"),
    move("move",&Laser_tilt::move_impl, &Laser_tilt::atpos_impl, this),
    port_prop("Device", "UNIX device file (/dev/ttySx)")
  {
    this->ports()->addPort(&position_port);

    this->commands()->addCommand(&move, "Set angle", "d", "Target angle");

    this->properties()->addProperty(&port_prop);
  }

  bool Laser_tilt::configureHook()
  {
   return true;
  }

  bool Laser_tilt::startHook()
  {
    Ourptz_settings set;
    set.offset = 227;
    set.pos_res = 1000;

    ptz = new Ourptz(port_prop.get().c_str() ,Ourptz_DEFAULT_BAUD ,set );

    return true;
  }

  void Laser_tilt::updateHook()
  {
    position = ptz->GetPos();
    position_port.Set(position);
  }

  void Laser_tilt::stopHook()
  {
    delete ptz;
  }

  void Laser_tilt::cleanupHook()
  {

  }

  bool Laser_tilt::move_impl(double d)
  {
	  target = d;
	  ptz->SetPos(target ,false);
	  return true;
  }

  bool Laser_tilt::atpos_impl(double d)
  {
	  return (position == target);
  }
}

ORO_CREATE_COMPONENT( orocos_test::Laser_tilt );
