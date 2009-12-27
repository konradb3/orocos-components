/*
 * Gazebo_laser.h
 *
 *  Created on: Dec 27, 2009
 *      Author: konrad
 */

#ifndef GAZEBO_LASER_H_
#define GAZEBO_LASER_H_

#include <vector>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Properties.hpp>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

namespace orocos_test
{

class Gazebo_laser: public RTT::TaskContext
{
public:
	Gazebo_laser(std::string name);

	/**
	 * This function is for the configuration code.
	 * Return false to abort configuration.
	 */
	bool configureHook();

	/**
	 * This function is for the application's start up code.
	 * Return false to abort start up.
	 */
	bool startHook();

	/**
	 * This function is called by the Execution Engine.
	 */
	void updateHook();

	/**
	 * This function is called when the task is stopped.
	 */
	void stopHook();

	/**
	 * This function is called when the task is being deconfigured.
	 */
	void cleanupHook();
protected:
	RTT::DataPort<std::vector<double> > distances_port, angles_port;
	RTT::Property<std::string> laserIface_prop;
private:
	  gazebo::Client *client;
	  gazebo::LaserIface *laserIface;

	  std::string laserIface_name;

	  std::vector<double> distances_local, angles_local;

	  int mens_count;
};

}

#endif /* GAZEBO_LASER_H_ */
