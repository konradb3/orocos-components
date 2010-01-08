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
    setpoint_port("Setpoint_input"),
    position_port("Position_output"),
    port_prop("Device", "UNIX device file (/dev/ttySx)")
  {
    this->ports()->addPort(&setpoint_port);
    this->ports()->addPort(&position_port);

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
    double setpoint, position;
    setpoint = setpoint_port.Get();
    ptz->SetPos(setpoint ,false);
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
}

ORO_CREATE_COMPONENT( orocos_test::Laser_tilt );
