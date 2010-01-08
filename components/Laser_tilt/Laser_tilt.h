/*
 * Laser_tilt.h
 *
 *  Created on: Jan 8, 2010
 *      Author: konradb3
 */

#ifndef LASER_TILT_H_
#define LASER_TILT_H_

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Properties.hpp>

#include "ourptz.h"

namespace orocos_test
{
  class Laser_tilt : public RTT::TaskContext
  {
  public:
    Laser_tilt(std::string name);
    /**
     * This function is for the configuration code.
     * Return false to abort configuration.
     */
    bool
    configureHook();

    /**
     * This function is for the application's start up code.
     * Return false to abort start up.
     */
    bool
    startHook();

    /**
     * This function is called by the Execution Engine.
     */
    void
    updateHook();

    /**
     * This function is called when the task is stopped.
     */
    void
    stopHook();

    /**
     * This function is called when the task is being deconfigured.
     */
    void
    cleanupHook();

  protected:
    RTT::DataPort<double> setpoint_port;
    RTT::DataPort<double> position_port;
    RTT::Property<std::string> port_prop;
  private:
    Ourptz *ptz;
  };

}

#endif /* LASER_TILT_H_ */
